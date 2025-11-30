#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

// Incluir páginas separadas
#include "pagina1.h"  // Página principal
#include "pagina2.h"  // Página setup
#include "pagina3.h"  // Página programación
#include "pagina4.h"  // Página de grupos
#include "pagina5.h"  // Página control panel
#include "pagina6.h"  // Editor de secuencias

// Credenciales del hotspot
const char* ssid = "Ramiro";
const char* password = "arduino9685";

const int PIR_SENSOR_PIN = 7;
bool motionDetected = false;

// Configuración de red
IPAddress local_ip(192,168,4,1);
IPAddress gateway(192,168,4,1);
IPAddress subnet(255,255,255,0);

WebServer server(80);

// Crear objetos para los dos PCA9685
Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pca2 = Adafruit_PWMServoDriver(0x41);

bool pca1_detected = false;
bool pca2_detected = false;


// === ESTRUCTURAS PRINCIPALES ===

const int MAX_BLOCKS = 50;  // Máximo número de bloques en secuencia

// Estructura para movimientos de servos
struct ServoMovement {
  int16_t angles[6];     // Ángulo destino (-90° a 90°)
  uint16_t times[6];     // Tiempo total del movimiento (ms) - INDEPENDIENTE
  uint16_t delays[6];    // Delay después del movimiento (ms)
  uint8_t velocities[6]; // Velocidad (0-100%) - INDEPENDIENTE
  uint8_t accelerations[6]; // Aceleración (0-100%)
  uint8_t stepCount;
};

// Estructura para guardar configuración de servos
struct ServoConfig {
  char name[20];
  bool enabled;
  ServoMovement movement;  // Rutina de movimientos
};


struct Group {
  char name[15];
  uint8_t servos[8];    // IDs de servos en el grupo (máx 8)
  uint8_t servoCount;
  bool enabled;
};

struct ProgramConfig {
  Group groups[6];       // Máx 6 grupos
  uint8_t groupCount;
  uint8_t executionMode; // 0=Secuencial, 1=Simultáneo, 2=Aleatorio
  uint8_t groupBehavior; // 0=Secuencial, 1=Simultáneo, 2=Aleatorio
};

struct ExecutionConfig {
  uint16_t loopInterval;
  bool useSensor;
  uint8_t currentMode;
};

struct TimelineBlock {
  uint8_t type;          // 0=servo, 1=grupo, 2=comportamiento
  uint8_t targetID;      // ID del servo o grupo
  char name[20];         // Nombre para mostrar
  uint16_t duration;     // Duración en ms
  uint8_t behavior;      // Comportamiento (0=secuencial, 1=simultáneo, 2=aleatorio)
};

struct Sequence {
  char sequenceName[20];
  TimelineBlock blocks[50];     // Máx 20 bloques
  uint8_t blockCount;
  uint16_t totalDuration;
  bool isLooping;
};



void executeSimultaneousGroup(Sequence &sequence, int startIndex, int endIndex);
void executeServoSimultaneous(uint8_t servoID, uint16_t duration);
void executeRandomInSequence(TimelineBlock block, uint16_t duration);
void executeGroupRandom(Group group, uint16_t duration);
void executeCurrentSequence();
void executeSequenceTest(Sequence &sequence);
// DECLARACIONES DE FUNCIONES - SOLO UNA VEZ
void executeServoMovementAdvanced(uint8_t servoID, ServoMovement movement, uint16_t maxDuration);
void executeAdvancedServoControl(uint8_t servoID, int targetAngle, unsigned long elapsedTime, unsigned long totalTime, uint8_t velocity, uint8_t acceleration);
void setServoPulse(uint8_t servoID, int pulse);
void executeServoInSequence(uint8_t servoID, uint8_t behavior, uint16_t maxDuration);



// === VARIABLES GLOBALES ===
ServoConfig servoConfigs[32];
ProgramConfig programConfig;
ExecutionConfig execConfig;
Sequence currentSequence;

// === VARIABLES GLOBALES ACTUALIZADAS ===
bool systemRunning = false;
bool loopEnabled = false;
bool sequenceMode = false;
unsigned long programStartTime = 0;
unsigned long lastLoopTime = 0;
unsigned long lastPIRTrigger = 0;
bool pirTriggered = false;
uint8_t currentExecutionStep = 0;
unsigned long executionStartTime = 0;
bool isExecutingSequence = false;
String currentExecutionInfo = "";

// ⬅️ NUEVO: Array para guardar la posición actual de cada servo
int currentServoPositions[32] = {0}; // Todos empiezan en 0°

// === NUEVAS VARIABLES PARA CONTROL PANEL ===
bool sequenceExecution = false;
unsigned long lastPIRTime = 0;
uint8_t currentStep = 0;
uint8_t totalSteps = 0;
Sequence executionSequence;



// === OFFSETS EEPROM (FIJOS) ===
const int EEPROM_SIZE = 5120;  // Aumentado para cubrir todos los offsets y tamaños calculados (~4323 + buffer)
const int PROGRAM_EEPROM_OFFSET = 32 * sizeof(ServoConfig);
const int EXEC_CONFIG_EEPROM_OFFSET = 2500;
const int SEQUENCE_EEPROM_OFFSET = 3000;

// === FUNCIONES ===
bool detectPCA9685(uint8_t address) {
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();
  return (error == 0);
}

// ⬅️ FUNCIÓN CORREGIDA PARA INJORA 35KG
int angleToPulseINJORA(int angle) {
  // Convertir -90° a 90° → 500μs a 2500μs (CORREGIDO)
  // INJORA 35KG usa 500-2500μs para -90° a 90°
  int us = map(angle, -90, 90, 500, 2500);
  // Convertir microsegundos a ticks PCA9685 (4096 steps, 20ms periodo)
  int pulse = (us * 4096) / 20000;
  
  // Limitar valores para seguridad
  if (pulse < 102) pulse = 102;     // 500μs mínimo
  if (pulse > 512) pulse = 512;     // 2500μs máximo
  
  return pulse;
}



// === NUEVA FUNCIÓN: Ejecutar secuencia guardada ===
void executeSavedSequence() {
  if (currentSequence.blockCount == 0) {
    Serial.println("❌ No hay secuencia guardada para ejecutar");
    return;
  }
  
  Serial.println("🎬 EJECUTANDO SECUENCIA GUARDADA");
  executeSequenceTest(currentSequence);
}

void executeCurrentSequence() {
  if (currentStep < totalSteps) {
    // Ejecutar bloque actual
    TimelineBlock currentBlock = executionSequence.blocks[currentStep];
    
    Serial.print("🎯 EJECUTANDO BLOQUE ");
    Serial.print(currentStep + 1);
    Serial.print("/");
    Serial.print(totalSteps);
    Serial.print(": ");
    Serial.println(currentBlock.name);
    
    // Actualizar progreso
    currentStep++;
    
    // Ejecutar el bloque
    executeSingleBlock(currentBlock);
    
  } else {
    // Secuencia completada
    Serial.println("✅ SECUENCIA COMPLETADA");
    
    if (execConfig.useSensor) {
      // Modo PIR: esperar siguiente señal
      sequenceExecution = false;
      Serial.println("⏳ ESPERANDO PRÓXIMA SEÑAL PIR...");
    } else {
      // Modo loop: reiniciar después del intervalo
      sequenceExecution = false;
      Serial.println("🔁 ESPERANDO INTERVALO PARA PRÓXIMO CICLO...");
    }
    
    // Resetear progreso
    currentStep = 0;
  }
}

void executeSingleBlock(TimelineBlock block) {
  // Crear una secuencia temporal con un solo bloque
  Sequence tempSequence;
  tempSequence.blockCount = 1;
  tempSequence.blocks[0] = block;
  
  // Ejecutar usando la función existente
  executeSequenceWithBehaviors(tempSequence);
}


void executeProgramLoop() {
  unsigned long currentTime = millis();
  
  // Verificar sensor PIR si está activado
  if (execConfig.useSensor) {
    checkPIRSensor();
    
    // Si PIR está activado y no hay ejecución en curso, iniciar
    if (pirTriggered && !sequenceExecution) {
      Serial.println("🚨 SEÑAL PIR DETECTADA - INICIANDO SECUENCIA");
      startSequenceExecution();
      pirTriggered = false;
    }
  }
  
  // Ejecutar secuencia si está activa
  if (sequenceExecution) {
    executeCurrentSequence();
  } 
  // Modo loop automático (sin PIR)
  else if (systemRunning && !execConfig.useSensor) {
    // Verificar intervalo entre ciclos
    if (currentTime - lastLoopTime >= execConfig.loopInterval) {
      Serial.println("🔁 INICIANDO CICLO AUTOMÁTICO");
      startSequenceExecution();
      lastLoopTime = currentTime;
    }
  }
}

void checkPIRSensor() {
  int pirState = digitalRead(PIR_SENSOR_PIN);
  
  if (pirState == HIGH) {
    if (!pirTriggered && (millis() - lastPIRTime > 1000)) { // Debounce de 1 segundo
      pirTriggered = true;
      lastPIRTime = millis();
      Serial.println("🚨 MOVIMIENTO DETECTADO POR PIR");
    }
  }
}

// === NUEVA FUNCIÓN: Iniciar ejecución de secuencia ===
void startSequenceExecution() {
  systemRunning = true;
  sequenceExecution = true;
  programStartTime = millis();
  lastLoopTime = millis();
  currentStep = 0;
  
  // Usar la secuencia actual para ejecución
  executionSequence = currentSequence;
  totalSteps = executionSequence.blockCount;
}

// Detener ejecución
void stopSequenceExecution() {
  systemRunning = false;
  sequenceExecution = false;
  pirTriggered = false;
  currentStep = 0;
  totalSteps = 0;
  
  // Mover todos los servos a posición central y apagar
  returnToCenterAndOff();
  
  Serial.println("⏹️ EJECUCIÓN DETENIDA");
}

void handleGetServoStatus() {
  String json = "[";
  for (int i = 0; i < 32; i++) {
    bool hasMovement = servoConfigs[i].movement.stepCount > 0;
    json += "{\"id\":" + String(i) + ",\"enabled\":" + (servoConfigs[i].enabled ? "true" : "false") + ",\"hasMovement\":" + (hasMovement ? "true" : "false") + "}";
    if (i < 31) json += ",";
  }
  json += "]";
  server.send(200, "application/json", json);
}

// === HANDLERS ===
void handleSequenceEditor() {
  server.send(200, "text/html", SEQUENCE_EDITOR_HTML);
}

void handleTestSequence() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    Serial.println("🎬 PROBANDO SECUENCIA TEMPORAL");
    
    DynamicJsonDocument doc(12288);
    DeserializationError error = deserializeJson(doc, body);
    
    if (error) {
      Serial.print("❌ Error parseando JSON: ");
      Serial.println(error.c_str());
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"JSON inválido\"}");
      return;
    }
    
    Sequence tempSequence;
    tempSequence.blockCount = 0;
    
    JsonArray blocks = doc["blocks"];
    for (JsonObject block : blocks) {
      if (tempSequence.blockCount >= 50) break;
      
      TimelineBlock tempBlock;
      
      const char* typeStr = block["type"];
      if (strcmp(typeStr, "servo") == 0) tempBlock.type = 0;
      else if (strcmp(typeStr, "group") == 0) tempBlock.type = 1;
      else if (strcmp(typeStr, "behavior") == 0) tempBlock.type = 2;
      // ⬅️ ELIMINADO: else if (strcmp(typeStr, "time") == 0) tempBlock.type = 3;  // Quitar tiempo
      
      tempBlock.targetID = block["targetID"];
      
      const char* nameStr = block["name"];
      strncpy(tempBlock.name, nameStr, 19);
      
      tempBlock.duration = block["duration"];
      
      if (block.containsKey("behavior") && !block["behavior"].isNull()) {
        const char* behaviorStr = block["behavior"];
        if (strcmp(behaviorStr, "sequential") == 0) tempBlock.behavior = 0;
        else if (strcmp(behaviorStr, "simultaneous") == 0) tempBlock.behavior = 1;
        else if (strcmp(behaviorStr, "random") == 0) tempBlock.behavior = 2;
        // ⬅️ NUEVO: Soporte para END
        else if (strcmp(behaviorStr, "end") == 0) tempBlock.behavior = 3;
      } else {
        tempBlock.behavior = 0;
      }
      
      tempSequence.blocks[tempSequence.blockCount] = tempBlock;
      tempSequence.blockCount++;
    }
    
    Serial.print("📦 Secuencia temporal: ");
    Serial.print(tempSequence.blockCount);
    Serial.println(" bloques");
    
    executeSequenceWithBehaviors(tempSequence);
    
    server.send(200, "application/json", "{\"status\":\"success\"}");
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Sin datos\"}");
  }
}

void executeSequenceWithBehaviors(Sequence &sequence) {
  Serial.println("🎬 SECUENCIA CON COMPORTAMIENTOS ACTIVOS");
  isExecutingSequence = true;

  uint8_t currentBehavior = 0; // Secuencial por defecto
  
  int groupStart = -1; // Inicio de grupo de bloques bajo un comportamiento
  
  for (int i = 0; i < sequence.blockCount; i++) {
    TimelineBlock block = sequence.blocks[i];
    
    if (block.type == 2) { // BLOQUE DE COMPORTAMIENTO
      // Si hay grupo pendiente, ejecutarlo antes de nuevo comportamiento
      if (groupStart != -1) {
        executeBehaviorGroup(sequence, groupStart, i - 1, currentBehavior);  // ⬅️ Sin currentDuration
        groupStart = -1;
      }
      
      if (block.behavior == 3) { // END - Termina grupo actual si hay
        if (groupStart != -1) {
          executeBehaviorGroup(sequence, groupStart, i - 1, currentBehavior);  // ⬅️ Sin currentDuration
          groupStart = -1;
        }
        Serial.println("🔚 BLOQUE END - Fin de grupo");
        continue;
      }
      
      currentBehavior = block.behavior;
      groupStart = i + 1; // Inicia nuevo grupo después del comportamiento
      Serial.print("🔄 COMPORTAMIENTO: ");
      if (currentBehavior == 0) Serial.println("SECUENCIAL");
      else if (currentBehavior == 1) Serial.println("SIMULTÁNEO");
      else if (currentBehavior == 2) Serial.println("ALEATORIO");
      continue;
    }
    
    // Si es servo/grupo y no hay grupo activo, asumir secuencial por defecto
    if (groupStart == -1 && (block.type == 0 || block.type == 1)) {
      groupStart = i;
      currentBehavior = 0;
    }
  }
  
  // Ejecutar último grupo pendiente
  if (groupStart != -1) {
    executeBehaviorGroup(sequence, groupStart, sequence.blockCount - 1, currentBehavior);  // ⬅️ Sin currentDuration
  }
  
  returnToCenterAndOff();
  Serial.println("✅ SECUENCIA CON COMPORTAMIENTOS COMPLETADA");
  isExecutingSequence = false;
}

// ⬅️ NUEVA FUNCIÓN: Obtener factor de velocidad desde slider (port de JS velocityCurve.sliderToFactor)
float getVelocityFactor(uint8_t sliderValue) {
  if (sliderValue <= 5) return 3.0f;   // Muy lento
  if (sliderValue <= 15) return 2.5f;
  if (sliderValue <= 30) return 2.0f;
  if (sliderValue <= 50) return 1.5f;
  if (sliderValue <= 70) return 1.2f;
  if (sliderValue <= 85) return 1.0f;
  return 0.8f;  // Rápido
}

uint16_t calculateServoDuration(uint8_t servoID) {
  if (servoID >= 32 || !servoConfigs[servoID].enabled) return 0;
  
  ServoMovement mov = servoConfigs[servoID].movement;
  uint32_t total = 0;  // Usar uint32_t para evitar overflow en sumas grandes
  for (uint8_t i = 0; i < mov.stepCount; i++) {
    float factor = getVelocityFactor(mov.velocities[i]);
    uint32_t effectiveTime = (uint32_t)(mov.times[i] * factor);
    total += effectiveTime + mov.delays[i];
  }
  return (total > 65535) ? 65535 : (uint16_t)total;  // Limitar a uint16_t max
}

// ⬅️ NUEVA FUNCIÓN: Calcular duración real de un grupo (máximo de sus servos, asumiendo paralelo)
uint16_t calculateGroupDuration(uint8_t groupID) {
  if (groupID >= programConfig.groupCount) return 0;
  
  Group group = programConfig.groups[groupID];
  uint16_t maxDuration = 0;
  
  for (uint8_t i = 0; i < group.servoCount; i++) {
    uint8_t servoID = group.servos[i];
    uint16_t servoDur = calculateServoDuration(servoID);
    if (servoDur > maxDuration) maxDuration = servoDur;
  }
  
  return maxDuration;
}

// Modificar executeBehaviorGroup para usar duraciones reales
void executeBehaviorGroup(Sequence &sequence, int startIndex, int endIndex, uint8_t behavior) {  // ⬅️ MODIFICADO: Quité duration param, calcular por bloque
  Serial.print("🛠️ EJECUTANDO GRUPO - Comportamiento: ");
  Serial.print(behavior);
  Serial.print(" | Bloques: ");
  Serial.print(startIndex);
  Serial.print("-");
  Serial.print(endIndex);
  Serial.println();

  if (behavior == 0) { // Secuencial
    for (int j = startIndex; j <= endIndex; j++) {
      TimelineBlock block = sequence.blocks[j];
      if (block.type == 0 || block.type == 1) {
        uint16_t realDuration = (block.type == 0) ? calculateServoDuration(block.targetID) : calculateGroupDuration(block.targetID);
        if (realDuration == 0) realDuration = 2000;  // Fallback si no programado
        
        Serial.print("  ⏩ Secuencial: ");
        Serial.print(block.name);
        Serial.print(" | Duración real: ");
        Serial.print(realDuration);
        Serial.println("ms");
        
        if (block.type == 0) {
          executeServoInSequence(block.targetID, behavior, realDuration); 
        } else {
          executeGroupInSequence(block.targetID, behavior, realDuration); 
        }
      }
    }
  } else if (behavior == 1) { // Simultáneo
    // Para simultáneo, calcular max de todos los bloques en el grupo
    uint16_t maxGroupDuration = 0;
    for (int j = startIndex; j <= endIndex; j++) {
      TimelineBlock block = sequence.blocks[j];
      if (block.type == 0 || block.type == 1) {
        uint16_t dur = (block.type == 0) ? calculateServoDuration(block.targetID) : calculateGroupDuration(block.targetID);
        if (dur > maxGroupDuration) maxGroupDuration = dur;
      }
    }
    if (maxGroupDuration == 0) maxGroupDuration = 2000;
    
    Serial.print(" | Duración grupo: ");
    Serial.print(maxGroupDuration);
    Serial.println("ms");
    
    executeSimultaneousGroup(sequence, startIndex, endIndex);  // Asumir que internamente usa tiempos reales
  } else if (behavior == 2) { // Aleatorio
    // Similar a secuencial, pero aleatorio, usar real por bloque
    int blockCount = endIndex - startIndex + 1;
    int randomOrder[blockCount];
    for (int k = 0; k < blockCount; k++) randomOrder[k] = startIndex + k;
    // Mezclar...
    for (int k = blockCount - 1; k > 0; k--) {
      int randIdx = random(k + 1);
      int temp = randomOrder[k];
      randomOrder[k] = randomOrder[randIdx];
      randomOrder[randIdx] = temp;
    }
    
    for (int k = 0; k < blockCount; k++) {
      TimelineBlock block = sequence.blocks[randomOrder[k]];
      if (block.type == 0 || block.type == 1) {
        uint16_t realDuration = (block.type == 0) ? calculateServoDuration(block.targetID) : calculateGroupDuration(block.targetID);
        if (realDuration == 0) realDuration = 2000;
        
        Serial.print("  🎲 Aleatorio: ");
        Serial.print(block.name);
        Serial.print(" | Duración real: ");
        Serial.print(realDuration);
        Serial.println("ms");
        
        if (block.type == 0) {
          executeServoInSequence(block.targetID, behavior, realDuration);
        } else {
          executeGroupInSequence(block.targetID, behavior, realDuration);
        }
      }
    }
  }
}

// === FUNCIÓN MEJORADA - COMPORTAMIENTOS CONSISTENTES ===

void executeSequenceTest(Sequence &sequence) {
  Serial.println("🎬 INICIANDO SECUENCIA - COMPORTAMIENTOS MEJORADOS");
  isExecutingSequence = true;

  uint16_t blockDurations[50] = {0};
  uint16_t currentDuration = 2000;
  uint8_t currentBehavior = 0;
  
  // PRIMERO: Calcular duraciones para cada bloque
  for (int i = 0; i < sequence.blockCount; i++) {
    TimelineBlock block = sequence.blocks[i];
    
    if (block.type == 3) { // BLOQUE DE TIEMPO
      currentDuration = block.duration;
      blockDurations[i] = currentDuration;
    } else {
      blockDurations[i] = currentDuration;
    }
  }
  
  // SEGUNDO: Ejecutar con comportamientos consistentes
  currentDuration = 2000;
  currentBehavior = 0;
  
  for (int i = 0; i < sequence.blockCount; i++) {
    TimelineBlock block = sequence.blocks[i];
    uint16_t blockTime = blockDurations[i];
    
    Serial.print("▶️ BLOQUE ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(block.name);
    Serial.print(" | Tipo: ");
    Serial.print(block.type);
    Serial.print(" | Tiempo: ");
    Serial.print(blockTime);
    Serial.print("ms | Comportamiento: ");
    Serial.println(currentBehavior);
    
    // ACTUALIZAR COMPORTAMIENTO ACTUAL
    if (block.type == 2) { // BLOQUE DE COMPORTAMIENTO
      currentBehavior = block.behavior;
      Serial.print("  🔄 NUEVO COMPORTAMIENTO: ");
      Serial.println(currentBehavior);
      continue;
    }
    
    // ACTUALIZAR DURACIÓN ACTUAL  
    if (block.type == 3) { // BLOQUE DE TIEMPO
      currentDuration = block.duration;
      continue;
    }
    
    // 🚀 DETECCIÓN MEJORADA DE SECUENCIAS REPETITIVAS - PARA MÚLTIPLES SERVOS
    bool shouldExecuteRepetitive = false;
    int timeBlockIndex = -1;
    uint8_t repetitiveBehavior = currentBehavior;
    int servoCount = 0;
    int servoStartIndex = i;

    // SOLO buscar patrones repetitivos si hay al menos 2 servos/grupos consecutivos
    for (int j = i; j < sequence.blockCount; j++) {
      if (sequence.blocks[j].type == 0 || sequence.blocks[j].type == 1) {
        servoCount++;
      } else {
        break; // Si encontramos otro tipo de bloque, detenemos
      }
    }

    // 🎯 SOLO activar modo repetitivo si hay 2 O MÁS servos/grupos consecutivos
    if (servoCount >= 2) {
      // Buscar el bloque de tiempo más cercano anterior
      for (int j = i - 1; j >= 0; j--) {
        if (sequence.blocks[j].type == 3) { // BLOQUE DE TIEMPO
          timeBlockIndex = j;
          // Buscar comportamiento anterior al tiempo
          for (int k = j - 1; k >= 0; k--) {
            if (sequence.blocks[k].type == 2) { // BLOQUE DE COMPORTAMIENTO
              repetitiveBehavior = sequence.blocks[k].behavior;
              break;
            }
          }
          break;
        }
      }
      
      // Si no encontramos tiempo específico, usar comportamiento actual
      if (timeBlockIndex == -1) {
        repetitiveBehavior = currentBehavior;
        shouldExecuteRepetitive = true;
      } else {
        shouldExecuteRepetitive = true;
      }
    }

    if (shouldExecuteRepetitive && servoCount >= 2) {
      uint16_t repetitiveTime = (timeBlockIndex != -1) ? 
          sequence.blocks[timeBlockIndex].duration : currentDuration;
      
      Serial.print("  🔁 COMPORTAMIENTO REPETITIVO: ");
      Serial.print("startIndex=");
      Serial.print(servoStartIndex);
      Serial.print(", servoCount=");
      Serial.print(servoCount);
      Serial.print(", comportamiento=");
      Serial.print(repetitiveBehavior);
      Serial.print(", tiempo=");
      Serial.print(repetitiveTime);
      Serial.println("ms");
      
      executeRepetitiveSequence(sequence, servoStartIndex, servoCount, repetitiveTime, repetitiveBehavior);
      
      // 🎯 SALTO CORRECTO: Saltar al ÚLTIMO servo procesado
      i = servoStartIndex + servoCount - 1;
      Serial.print("  ➡️ Saltando al índice: ");
      Serial.println(i);
      continue; // Importantísimo: saltar a siguiente iteración
    }

    // 🎯 EJECUCIÓN NORMAL PARA BLOQUES INDIVIDUALES O SECUENCIAS DE 1 ELEMENTO
    Serial.print("  🎯 EJECUTANDO BLOQUE INDIVIDUAL: ");
    Serial.println(block.name);
    
    if (block.type == 0) { // SERVO INDIVIDUAL
      if (currentBehavior == 0) { // SECUENCIAL
        executeServoInSequence(block.targetID, currentBehavior, blockTime);
      } 
      else if (currentBehavior == 1) { // SIMULTÁNEO
        executeServoSimultaneous(block.targetID, blockTime);
      }
      else if (currentBehavior == 2) { // ALEATORIO
        executeRandomInSequence(block, blockTime);
      }
    } 
    else if (block.type == 1) { // GRUPO
      executeGroupInSequence(block.targetID, currentBehavior, blockTime);
    }
  }
  
  returnToCenterAndOff();
  Serial.println("✅ FIN SECUENCIA - COMPORTAMIENTOS APLICADOS CORRECTAMENTE");
  isExecutingSequence = false;
}




void handleGetExecutionInfo() {
  String json = "{";
  json += "\"running\":";
  json += systemRunning ? "true" : "false";
  json += ",\"executionTime\":";
  json += systemRunning ? String(millis() - executionStartTime) : "0";
  json += ",\"currentInfo\":\"";
  
  if (systemRunning && isExecutingSequence) {
    json += currentExecutionInfo;
  } else if (systemRunning) {
    json += "🔄 Esperando próximo ciclo...";
  } else {
    json += "⏹️ Sistema detenido";
  }
  
  json += "\",\"sequenceName\":\"";
  json += String(currentSequence.sequenceName);
  json += "\"}";
  
  server.send(200, "application/json", json);
}

void executeRepetitiveSequence(Sequence &sequence, int startIndex, int servoCount, uint16_t totalTime, uint8_t behavior) {
  Serial.print("  🔄 INICIANDO SECUENCIA REPETITIVA - COMPORTAMIENTO: ");
  Serial.println(behavior);
  
  unsigned long startTime = millis();
  unsigned long elapsed = 0;
  int cycleCount = 0;
  
  if (behavior == 0) { // SECUENCIAL - ⬅️ MODIFICACIÓN APROBADA
    // CALCULAR TIEMPO TOTAL DE UN CICLO COMPLETO (suma de todos los tiempos individuales)
    unsigned long totalCycleTime = 0;
    unsigned long individualTimes[servoCount];
    
    for (int i = 0; i < servoCount; i++) {
      TimelineBlock block = sequence.blocks[startIndex + i];
      individualTimes[i] = 0;
      
      if (block.type == 0) { // SERVO INDIVIDUAL
        ServoMovement movement = servoConfigs[block.targetID].movement;
        for (int step = 0; step < movement.stepCount; step++) {
          individualTimes[i] += movement.times[step];
        }
        totalCycleTime += individualTimes[i];
        
        Serial.print("    Servo ");
        Serial.print(block.targetID);
        Serial.print(" - ");
        Serial.print(block.name);
        Serial.print(": ");
        Serial.print(individualTimes[i]);
        Serial.println("ms por ciclo completo");
      }
      else if (block.type == 1) { // GRUPO
        // Para grupos, calcular tiempo del grupo completo
        Group group = programConfig.groups[block.targetID];
        unsigned long groupTime = 0;
        for (int j = 0; j < group.servoCount; j++) {
          ServoMovement movement = servoConfigs[group.servos[j]].movement;
          for (int step = 0; step < movement.stepCount; step++) {
            groupTime += movement.times[step];
          }
        }
        individualTimes[i] = groupTime;
        totalCycleTime += individualTimes[i];
      }
    }
    
    Serial.print("    Tiempo por vuelta secuencial completa: ");
    Serial.print(totalCycleTime);
    Serial.println("ms");
    
    // EJECUTAR CICLOS HASTA AGOTAR EL TIEMPO TOTAL
    while (elapsed < totalTime) {
      cycleCount++;
      Serial.print("    VUELTA SECUENCIAL ");
      Serial.print(cycleCount);
      Serial.print(" - Tiempo restante: ");
      Serial.print(totalTime - elapsed);
      Serial.println("ms");
      
      // EJECUTAR CADA SERVO EN SECUENCIA
      for (int i = 0; i < servoCount && elapsed < totalTime; i++) {
        TimelineBlock block = sequence.blocks[startIndex + i];
        unsigned long timeForThisServo = individualTimes[i];
        
        // ⬅️ NUEVO: AJUSTAR TIEMPO SI ES EL ÚLTIMO CICLO Y NO CABE COMPLETO
        unsigned long timeRemaining = totalTime - elapsed;
        if (timeForThisServo > timeRemaining) {
          timeForThisServo = timeRemaining;
        }
        
        Serial.print("      🎭 EJECUTANDO: ");
        Serial.print(block.name);
        Serial.print(" - Tiempo asignado: ");
        Serial.print(timeForThisServo);
        Serial.println("ms");
        
        if (block.type == 0) {
          executeServoInSequence(block.targetID, 0, timeForThisServo);
        } 
        else if (block.type == 1) {
          executeGroupInSequence(block.targetID, 0, timeForThisServo);
        }
        
        elapsed = millis() - startTime;
        if (elapsed >= totalTime) break;
      }
      
      elapsed = millis() - startTime;
    }
  }
  else if (behavior == 1) { // SIMULTÁNEO - ⬅️ IMPLEMENTACIÓN COMPLETA
    Serial.println("      🔄 COMPORTAMIENTO: SIMULTÁNEO REAL");
    
    // OBTENER LISTA DE TODOS LOS SERVOS A EJECUTAR EN PARALELO
    int totalServos = 0;
    int servoList[32]; // Máximo 32 servos
    ServoMovement movements[32];
    
    for (int i = 0; i < servoCount; i++) {
        TimelineBlock block = sequence.blocks[startIndex + i];
        
        if (block.type == 0) { // SERVO INDIVIDUAL
            if (block.targetID < 32 && servoConfigs[block.targetID].enabled) {
                servoList[totalServos] = block.targetID;
                movements[totalServos] = servoConfigs[block.targetID].movement;
                totalServos++;
                Serial.print("        🎭 SERVO SIMULTÁNEO: ");
                Serial.println(block.name);
            }
        } else if (block.type == 1) { // GRUPO
            Group group = programConfig.groups[block.targetID];
            Serial.print("        👥 GRUPO SIMULTÁNEO: ");
            Serial.println(block.name);
            
            for (int j = 0; j < group.servoCount; j++) {
                int servoID = group.servos[j];
                if (servoID < 32 && servoConfigs[servoID].enabled) {
                    servoList[totalServos] = servoID;
                    movements[totalServos] = servoConfigs[servoID].movement;
                    totalServos++;
                    Serial.print("          Servo ");
                    Serial.println(servoID);
                }
            }
        }
    }
    
    if (totalServos == 0) {
        Serial.println("        ⚠️  No hay servos para ejecutar");
        return;
    }
    
    Serial.print("        Total servos en paralelo: ");
    Serial.println(totalServos);
    
    // EJECUTAR EN PARALELO DURANTE EL TIEMPO TOTAL
    unsigned long startTime = millis();
    unsigned long elapsed = 0;
    
    while (elapsed < totalTime) {
        // PARA CADA SERVO, CALCULAR Y EJECUTAR SU POSICIÓN ACTUAL BASADA EN EL TIEMPO TRANSCURRIDO
        for (int i = 0; i < totalServos; i++) {
            ServoMovement movement = movements[i];
            if (movement.stepCount == 0) continue;
            
            // CALCULAR TIEMPO TOTAL DE UN CICLO PARA ESTE SERVO
            unsigned long totalMovementTime = 0;
            for (int step = 0; step < movement.stepCount; step++) {
                totalMovementTime += movement.times[step];
            }
            
            if (totalMovementTime > 0) {
                // CALCULAR PROGRESO DENTRO DEL CICLO ACTUAL
                unsigned long cycleProgress = elapsed % totalMovementTime;
                unsigned long accumulatedTime = 0;
                int currentStep = 0;
                
                // ENCONTRAR EL PASO ACTUAL BASADO EN EL TIEMPO TRANSCURRIDO
                for (int step = 0; step < movement.stepCount; step++) {
                    accumulatedTime += movement.times[step];
                    if (cycleProgress < accumulatedTime) {
                        currentStep = step;
                        break;
                    }
                }
                
                // MOVER EL SERVO A LA POSICIÓN DEL PASO ACTUAL
                int angle = movement.angles[currentStep];
                int pulse = angleToPulseINJORA(angle);
                
                if (servoList[i] < 16 && pca1_detected) {
                    pca1.setPWM(servoList[i], 0, pulse);
                } else if (servoList[i] >= 16 && pca2_detected) {
                    pca2.setPWM(servoList[i] - 16, 0, pulse);
                }
                
                Serial.print("        Servo ");
                Serial.print(servoList[i]);
                Serial.print(" - Paso ");
                Serial.print(currentStep);
                Serial.print(": ");
                Serial.print(angle);
                Serial.println("°");
            }
        }
        
        // PEQUEÑA PAUSA PARA NO SATURAR
        delay(50);
        elapsed = millis() - startTime;
    }
    
    Serial.println("      ✅ SIMULTÁNEO COMPLETADO");
}
else if (behavior == 2) { // ALEATORIO - ⬅️ IMPLEMENTACIÓN COMPLETA
    Serial.println("      🎲 COMPORTAMIENTO: ALEATORIO");
    
    // CALCULAR TIEMPO TOTAL DE UN CICLO COMPLETO (suma de todos los tiempos individuales)
    unsigned long totalCycleTime = 0;
    unsigned long individualTimes[servoCount];
    int servoIndices[servoCount]; // Para almacenar los índices originales
    
    for (int i = 0; i < servoCount; i++) {
        TimelineBlock block = sequence.blocks[startIndex + i];
        servoIndices[i] = i; // Guardar índice original
        individualTimes[i] = 0;
        
        if (block.type == 0) { // SERVO INDIVIDUAL
            ServoMovement movement = servoConfigs[block.targetID].movement;
            for (int step = 0; step < movement.stepCount; step++) {
                individualTimes[i] += movement.times[step];
            }
            totalCycleTime += individualTimes[i];
            
            Serial.print("        Servo ");
            Serial.print(block.targetID);
            Serial.print(" - ");
            Serial.print(block.name);
            Serial.print(": ");
            Serial.print(individualTimes[i]);
            Serial.println("ms por ciclo completo");
        }
        else if (block.type == 1) { // GRUPO
            Group group = programConfig.groups[block.targetID];
            unsigned long groupTime = 0;
            for (int j = 0; j < group.servoCount; j++) {
                ServoMovement movement = servoConfigs[group.servos[j]].movement;
                for (int step = 0; step < movement.stepCount; step++) {
                    groupTime += movement.times[step];
                }
            }
            individualTimes[i] = groupTime;
            totalCycleTime += groupTime;
        }
    }
    
    Serial.print("    Tiempo por vuelta aleatoria completa: ");
    Serial.print(totalCycleTime);
    Serial.println("ms");
    
    // EJECUTAR CICLOS HASTA AGOTAR EL TIEMPO TOTAL
    unsigned long startTime = millis();
    unsigned long elapsed = 0;
    int cycleCount = 0;
    
    while (elapsed < totalTime) {
        cycleCount++;
        Serial.print("    VUELTA ALEATORIA ");
        Serial.print(cycleCount);
        Serial.print(" - Tiempo restante: ");
        Serial.print(totalTime - elapsed);
        Serial.println("ms");
        
        // CREAR ORDEN ALEATORIO PARA ESTE CICLO
        int randomOrder[servoCount];
        for (int i = 0; i < servoCount; i++) {
            randomOrder[i] = i;
        }
        
        // MEZCLAR EL ORDEN (Algoritmo Fisher-Yates)
        for (int i = servoCount - 1; i > 0; i--) {
            int j = random(i + 1);
            int temp = randomOrder[i];
            randomOrder[i] = randomOrder[j];
            randomOrder[j] = temp;
        }
        
        Serial.print("      🎲 Orden aleatorio: ");
        for (int i = 0; i < servoCount; i++) {
            Serial.print(randomOrder[i]);
            if (i < servoCount - 1) Serial.print(", ");
        }
        Serial.println();
        
        // EJECUTAR SERVOS EN ORDEN ALEATORIO
        for (int i = 0; i < servoCount && elapsed < totalTime; i++) {
            int originalIndex = randomOrder[i];
            TimelineBlock block = sequence.blocks[startIndex + originalIndex];
            unsigned long timeForThisServo = individualTimes[originalIndex];
            
            // AJUSTAR TIEMPO SI ES EL ÚLTIMO CICLO Y NO CABE COMPLETO
            unsigned long timeRemaining = totalTime - elapsed;
            if (timeForThisServo > timeRemaining) {
                timeForThisServo = timeRemaining;
            }
            
            Serial.print("      🎭 EJECUTANDO [Aleatorio]: ");
            Serial.print(block.name);
            Serial.print(" - Tiempo asignado: ");
            Serial.print(timeForThisServo);
            Serial.println("ms");
            
            if (block.type == 0) {
                executeServoInSequence(block.targetID, 0, timeForThisServo);
            } 
            else if (block.type == 1) {
                executeGroupInSequence(block.targetID, 0, timeForThisServo);
            }
            
            elapsed = millis() - startTime;
            if (elapsed >= totalTime) break;
        }
        
        elapsed = millis() - startTime;
    }
    
    Serial.print("      ✅ ALEATORIO COMPLETADO: ");
    Serial.print(cycleCount);
    Serial.print(" vueltas, ");
    Serial.print(elapsed);
    Serial.print("ms de ");
    Serial.print(totalTime);
    Serial.println("ms");
}
  
  Serial.print("  ✅ SECUENCIA REPETITIVA COMPLETADA: ");
  Serial.print(cycleCount);
  Serial.print(" vueltas, Comportamiento: ");
  Serial.print(behavior);
  Serial.print(", ");
  Serial.print(elapsed);
  Serial.print("ms de ");
  Serial.print(totalTime);
  Serial.println("ms");
}

void executeServoSimultaneous(uint8_t servoID, uint16_t duration) {
  if (servoID < 32 && servoConfigs[servoID].enabled) {
    Serial.print("  🔄 SERVO SIMULTÁNEO INDIVIDUAL: ");
    Serial.print(servoID);
    Serial.print(" - ");
    Serial.print(servoConfigs[servoID].name);
    Serial.print(" | Duración: ");
    Serial.print(duration);
    Serial.println("ms");
    
    executeServoInSequence(servoID, 0, duration); // Usar secuencial interno para el servo
  }
}

void executeSimultaneousGroup(Sequence &sequence, int startIndex, int endIndex, uint16_t duration) {
  Serial.print("🔄 EJECUTANDO SIMULTÁNEO REAL: Bloques ");
  Serial.print(startIndex);
  Serial.print("-");
  Serial.print(endIndex);
  Serial.print(" | Duración: ");
  Serial.print(duration);
  Serial.println("ms");
  
  // CONTAR CUÁNTOS SERVOS HAY QUE EJECUTAR EN PARALELO
  int totalServos = 0;
  int servoList[32]; // Máximo 32 servos
  
  for (int i = startIndex; i <= endIndex; i++) {
    TimelineBlock block = sequence.blocks[i];
    if (block.type == 0) { // SERVO INDIVIDUAL
      servoList[totalServos] = block.targetID;
      totalServos++;
      Serial.print("  🎯 Servo simultáneo: ");
      Serial.println(block.targetID);
    } else if (block.type == 1) { // GRUPO
      Group group = programConfig.groups[block.targetID];
      for (int j = 0; j < group.servoCount; j++) {
        servoList[totalServos] = group.servos[j];
        totalServos++;
        Serial.print("  🎯 Servo de grupo: ");
        Serial.println(group.servos[j]);
      }
    }
  }
  
  if (totalServos == 0) {
    Serial.println("  ⚠️ No hay servos para ejecutar en simultáneo");
    return;
  }
  
  Serial.print("  🔄 Ejecutando ");
  Serial.print(totalServos);
  Serial.println(" servos en paralelo");
  
  // 🆕 EJECUTAR TODOS LOS SERVOS EN PARALELO
  unsigned long startTime = millis();
  
  while (millis() - startTime < duration) {
    // PARA CADA SERVO, EJECUTAR UN PEQUEÑO AVANCE
    for (int i = 0; i < totalServos; i++) {
      // 🆕 AQUÍ NECESITAMOS UNA FUNCIÓN QUE EJECUTE UN PEQUEÑO PASO DE CADA SERVO
      // POR AHORA USAREMOS LA EXISTENTE PERO CON TIEMPO MUY CORTO
      executeServoInSequence(servoList[i], 0, 100); // 100ms por ciclo
    }
    delay(50); // Pequeña pausa
  }
  
  Serial.println("  ✅ SIMULTÁNEO COMPLETADO");
}

void executeSimultaneousGroupWithMaxTime(Sequence &sequence, int startIndex, int endIndex, uint16_t maxTotalDuration) {
  Serial.print("🔄 EJECUTANDO VERDADERO PARALELISMO CON TIEMPOS INDIVIDUALES ");
  Serial.print(startIndex);
  Serial.print(" a ");
  Serial.print(endIndex);
  Serial.print(" - DURACIÓN TOTAL: ");
  Serial.print(maxTotalDuration);
  Serial.println("ms");
  
  // OBTENER LISTA DE SERVOS CON SUS TIEMPOS INDIVIDUALES
  int totalServos = 0;
  int servoList[32];
  ServoMovement movements[32];
  uint16_t servoDurations[32];
  unsigned long servoStartTimes[32]; // ⬅️ NUEVO: Tiempo de inicio de cada servo
  
  for (int i = startIndex; i <= endIndex; i++) {
    TimelineBlock block = sequence.blocks[i];
    
    if (block.type == 0) {
      if (block.targetID < 32 && servoConfigs[block.targetID].enabled) {
        servoList[totalServos] = block.targetID;
        movements[totalServos] = servoConfigs[block.targetID].movement;
        servoDurations[totalServos] = block.duration;
        totalServos++;
      }
    } else if (block.type == 1) {
      Group group = programConfig.groups[block.targetID];
      for (int j = 0; j < group.servoCount; j++) {
        int servoID = group.servos[j];
        if (servoID < 32 && servoConfigs[servoID].enabled) {
          servoList[totalServos] = servoID;
          movements[totalServos] = servoConfigs[servoID].movement;
          servoDurations[totalServos] = block.duration;
          totalServos++;
        }
      }
    }
  }
  
  if (totalServos == 0) return;
  
  unsigned long globalStartTime = millis();
  
  // INICIALIZAR TODOS LOS SERVOS AL MISMO TIEMPO
  for (int i = 0; i < totalServos; i++) {
    servoStartTimes[i] = globalStartTime;
  }
  
  // EJECUTAR PARALELAMENTE HASTA EL TIEMPO TOTAL
  while (millis() - globalStartTime < maxTotalDuration) {
    bool allServosFinished = true;
    
    // EJECUTAR CADA SERVO EN PARALELO SEGÚN SU TIEMPO INDIVIDUAL
    for (int i = 0; i < totalServos; i++) {
      unsigned long servoElapsed = millis() - servoStartTimes[i];
      
      // ⬅️ SOLO EJECUTAR SI EL SERVO TIENE TIEMPO RESTANTE
      if (servoElapsed < servoDurations[i]) {
        allServosFinished = false;
        
        // Calcular progreso del ciclo para este servo
        ServoMovement movement = movements[i];
        if (movement.stepCount > 0) {
          unsigned long totalMovementTime = 0;
          for (int s = 0; s < movement.stepCount; s++) {
            totalMovementTime += movement.times[s];
          }
          
          // Calcular paso actual basado en el tiempo transcurrido
          unsigned long progress = servoElapsed % totalMovementTime;
          unsigned long accumulatedTime = 0;
          int currentStep = 0;
          
          // Encontrar paso actual
          for (int s = 0; s < movement.stepCount; s++) {
            accumulatedTime += movement.times[s];
            if (progress < accumulatedTime) {
              currentStep = s;
              break;
            }
          }
          
          // Mover servo al paso actual
          int angle = movement.angles[currentStep];
          int pulse = angleToPulseINJORA(angle);
          
          if (servoList[i] < 16 && pca1_detected) {
            pca1.setPWM(servoList[i], 0, pulse);
          } else if (servoList[i] >= 16 && pca2_detected) {
            pca2.setPWM(servoList[i] - 16, 0, pulse);
          }
        }
      }
    }
    
    if (allServosFinished) break;
    delay(50); // Pequeña pausa para no saturar
  }
  
  Serial.println("  ✅ PARALELISMO CON TIEMPOS INDIVIDUALES COMPLETADO");
}


void returnToCenterAndOff() {
  Serial.println("🔄 Volviendo a posición central...");
  for (int i = 0; i < 32; i++) {
    if (servoConfigs[i].enabled) {
      int centerPulse = angleToPulseINJORA(0);
      if (i < 16 && pca1_detected) {
        pca1.setPWM(i, 0, centerPulse);
      } else if (i >= 16 && i < 32 && pca2_detected) {
        pca2.setPWM(i - 16, 0, centerPulse);
      }
      // ⬅️ ACTUALIZAR POSICIÓN ACTUAL
      currentServoPositions[i] = 0;
    }
  }
  delay(500);
  
  // Apagar servos
  for (int i = 0; i < 32; i++) {
    if (servoConfigs[i].enabled) {
      if (i < 16 && pca1_detected) {
        pca1.setPWM(i, 0, 0);
      } else if (i >= 16 && i < 32 && pca2_detected) {
        pca2.setPWM(i - 16, 0, 0);
      }
    }
  }
}

void executeGroupSequential(Group group, uint16_t maxDuration) {
  Serial.println("      ⏩ COMPORTAMIENTO: SECUENCIAL CON TIEMPO MÁXIMO");
  Serial.print("      👥 Grupo: ");
  Serial.print(group.name);
  Serial.print(" | Servos: ");
  Serial.print(group.servoCount);
  Serial.print(" | Tiempo máximo: ");
  Serial.print(maxDuration);
  Serial.println("ms");
  
  unsigned long startTime = millis();
  unsigned long elapsed = 0;
  int cycleCount = 0;
  
  // CALCULAR TIEMPO POR SERVO BASADO EN EL TIEMPO MÁXIMO
  unsigned long timePerServo = maxDuration / group.servoCount;
  Serial.print("      ⏱️ Tiempo por servo: ");
  Serial.print(timePerServo);
  Serial.println("ms");
  
  // EJECUTAR SERVIDOS EN SECUENCIA HASTA AGOTAR EL TIEMPO MÁXIMO
  while (elapsed < maxDuration) {
    cycleCount++;
    Serial.print("        🔁 Ciclo grupo ");
    Serial.print(cycleCount);
    Serial.print(" - Tiempo restante: ");
    Serial.print(maxDuration - elapsed);
    Serial.println("ms");
    
    for (int i = 0; i < group.servoCount; i++) {
      elapsed = millis() - startTime;
      if (elapsed >= maxDuration) break;
      
      Serial.print("          🎭 Ejecutando: Servo ");
      Serial.println(group.servos[i]);
      
      // Ejecutar servo actual con su tiempo asignado
      executeServoInSequence(group.servos[i], 0, timePerServo);
      
      elapsed = millis() - startTime;
      if (elapsed >= maxDuration) break;
    }
    
    elapsed = millis() - startTime;
  }
  
  Serial.print("      ✅ GRUPO SECUENCIAL COMPLETADO: ");
  Serial.print(elapsed);
  Serial.print("ms de ");
  Serial.print(maxDuration);
  Serial.println("ms");
}

void executeGroupSimultaneous(Group group, uint16_t duration) {
  Serial.println("      🔄 COMPORTAMIENTO: SIMULTÁNEO EXACTO - REPRODUCCIÓN FIEL");
  
  // 🟢 USAR REPRODUCCIÓN EXACTA EN LUGAR DEL CÓDIGO ANTERIOR
  // Buscar el ID del grupo en programConfig
  for (int i = 0; i < programConfig.groupCount; i++) {
    if (strcmp(programConfig.groups[i].name, group.name) == 0) {
      executeGroupExactSimultaneous(i);
      return;
    }
  }
  
  // Fallback al método anterior si no se encuentra
  Serial.println("      ⚠️  Grupo no encontrado, usando método legacy");
  executeGroupSequential(group, duration);
}

// === REEMPLAZAR executeSimultaneousGroup con verdadero paralelismo ===
void executeSimultaneousGroup(Sequence &sequence, int startIndex, int endIndex) {
  Serial.print("🔄 EJECUTANDO BLOQUES SIMULTÁNEOS ");
  Serial.print(startIndex);
  Serial.print(" a ");
  Serial.print(endIndex);
  Serial.println("");
  
  // ENCONTRAR LA DURACIÓN MÁXIMA ENTRE LOS BLOQUES
  uint16_t maxDuration = 0;
  for (int i = startIndex; i <= endIndex; i++) {
    if (sequence.blocks[i].duration > maxDuration) {
      maxDuration = sequence.blocks[i].duration;
    }
  }
  
  Serial.print("  ⏱️ Duración simultánea: ");
  Serial.print(maxDuration);
  Serial.println("ms");
  
  // OBTENER LISTA DE TODOS LOS SERVOS A EJECUTAR EN PARALELO
  int totalServos = 0;
  int servoList[32]; // Máximo 32 servos
  ServoMovement movements[32];
  
  for (int i = startIndex; i <= endIndex; i++) {
    TimelineBlock block = sequence.blocks[i];
    
    if (block.type == 0) { // SERVO INDIVIDUAL
      if (block.targetID < 32 && servoConfigs[block.targetID].enabled) {
        servoList[totalServos] = block.targetID;
        movements[totalServos] = servoConfigs[block.targetID].movement;
        totalServos++;
        Serial.print("    🎭 SERVO SIMULTÁNEO: ");
        Serial.println(block.name);
      }
    } else if (block.type == 1) { // GRUPO
      Group group = programConfig.groups[block.targetID];
      Serial.print("    👥 GRUPO SIMULTÁNEO: ");
      Serial.println(block.name);
      
      for (int j = 0; j < group.servoCount; j++) {
        int servoID = group.servos[j];
        if (servoID < 32 && servoConfigs[servoID].enabled) {
          servoList[totalServos] = servoID;
          movements[totalServos] = servoConfigs[servoID].movement;
          totalServos++;
          Serial.print("      Servo ");
          Serial.println(servoID);
        }
      }
    }
  }
  
  if (totalServos == 0) {
    Serial.println("    ⚠️  No hay servos para ejecutar");
    return;
  }
  
  Serial.print("    Total servos en paralelo: ");
  Serial.println(totalServos);
  
  // EJECUTAR EN PARALELO PASO A PASO
  unsigned long startTime = millis();
  unsigned long elapsed = 0;
  int cycleCount = 0;
  
  while (elapsed < maxDuration) {
    cycleCount++;
    Serial.print("    🔄 Ciclo paralelo ");
    Serial.println(cycleCount);
    
    // EJECUTAR TODOS LOS PASOS DE TODOS LOS SERVOS EN PARALELO
    bool allServosFinished = false;
    int currentStep[32] = {0}; // Paso actual para cada servo
    bool servoActive[32]; // Si el servo aún tiene pasos
    
    for (int i = 0; i < totalServos; i++) {
      servoActive[i] = (movements[i].stepCount > 0);
    }
    
    // EJECUTAR PASOS HASTA QUE TODOS LOS SERVOS TERMINEN O SE AGOTE EL TIEMPO
    while (!allServosFinished && elapsed < maxDuration) {
      allServosFinished = true;
      int maxStepTime = 0;
      
      // MOVER TODOS LOS SERVOS A SU POSICIÓN ACTUAL
      for (int i = 0; i < totalServos; i++) {
        if (servoActive[i] && currentStep[i] < movements[i].stepCount) {
          allServosFinished = false;
          
          int angle = movements[i].angles[currentStep[i]];
          int stepTime = movements[i].times[currentStep[i]];
          int pulse = angleToPulseINJORA(angle);
          
          // ACTUALIZAR TIEMPO MÁXIMO PARA ESTE PASO
          if (stepTime > maxStepTime) {
            maxStepTime = stepTime;
          }
          
          // MOVER EL SERVO
          if (servoList[i] < 16 && pca1_detected) {
            pca1.setPWM(servoList[i], 0, pulse);
          } else if (servoList[i] >= 16 && pca2_detected) {
            pca2.setPWM(servoList[i] - 16, 0, pulse);
          }
          
          Serial.print("      Servo ");
          Serial.print(servoList[i]);
          Serial.print(" - Paso ");
          Serial.print(currentStep[i]);
          Serial.print(": ");
          Serial.print(angle);
          Serial.print("° (");
          Serial.print(stepTime);
          Serial.println("ms)");
        }
      }
      
      if (allServosFinished) break;
      
      // ESPERAR EL TIEMPO MÁXIMO DEL PASO ACTUAL
      if (maxStepTime > 0) {
        unsigned long stepStart = millis();
        while (millis() - stepStart < maxStepTime) {
          elapsed = millis() - startTime;
          if (elapsed >= maxDuration) break;
          delay(10);
        }
      }
      
      // AVANZAR AL SIGUIENTE PASO PARA TODOS LOS SERVOS
      for (int i = 0; i < totalServos; i++) {
        if (servoActive[i] && currentStep[i] < movements[i].stepCount) {
          currentStep[i]++;
          if (currentStep[i] >= movements[i].stepCount) {
            servoActive[i] = false; // Este servo terminó su ciclo
          }
        }
      }
      
      elapsed = millis() - startTime;
      if (elapsed >= maxDuration) break;
    }
    
    elapsed = millis() - startTime;
  }
  
  Serial.println("  ✅ PARALELISMO COMPLETADO");
}



// === NUEVA FUNCIÓN: Ejecutar en modo aleatorio ===
void executeRandomInSequence(TimelineBlock block, uint16_t maxDuration) {
  if (block.type == 0) { // SERVO INDIVIDUAL
    // Para servo individual en modo aleatorio, ejecutar normal con tiempo máximo
    executeServoInSequence(block.targetID, 2, maxDuration);
  } else if (block.type == 1) { // GRUPO
    // Para grupo en modo aleatorio, ejecutar servos en orden aleatorio con tiempo máximo
    Group group = programConfig.groups[block.targetID];
    
    Serial.print("    🎲 GRUPO ALEATORIO CON TIEMPO MÁXIMO: ");
    Serial.print(group.name);
    Serial.print(" | Servos: ");
    Serial.print(group.servoCount);
    Serial.print(" | Tiempo máximo: ");
    Serial.print(maxDuration);
    Serial.println("ms");
    
    // CREAR ORDEN ALEATORIO
    int randomOrder[group.servoCount];
    for (int i = 0; i < group.servoCount; i++) {
      randomOrder[i] = group.servos[i];
    }
    
    // MEZCLAR EL ORDEN
    for (int i = 0; i < group.servoCount; i++) {
      int randomIndex = random(group.servoCount);
      int temp = randomOrder[i];
      randomOrder[i] = randomOrder[randomIndex];
      randomOrder[randomIndex] = temp;
    }
    
    // CALCULAR TIEMPO POR SERVO BASADO EN EL TIEMPO MÁXIMO
    uint16_t timePerServo = maxDuration / group.servoCount;
    Serial.print("      ⏱️ Tiempo por servo aleatorio: ");
    Serial.print(timePerServo);
    Serial.println("ms");
    
    // EJECUTAR EN ORDEN ALEATORIO
    for (int i = 0; i < group.servoCount; i++) {
      Serial.print("        🎲 Servo aleatorio: ");
      Serial.println(randomOrder[i]);
      executeServoInSequence(randomOrder[i], 0, timePerServo);
    }
  }
}


// 🧠 FUNCIÓN INTELIGENTE - COMPATIBLE CON CONTROL PANEL
// 🧠 FUNCIÓN INTELIGENTE ACTUALIZADA
void executeServoInSequence(uint8_t servoID, uint8_t behavior, uint16_t maxDuration) {
  if (servoID >= 32 || !servoConfigs[servoID].enabled) {
    Serial.print("    ❌ SERVO ");
    Serial.print(servoID);
    Serial.println(" NO DISPONIBLE");
    return;
  }

  ServoMovement movement = servoConfigs[servoID].movement;
  
  if (movement.stepCount == 0) {
    Serial.println("    ⚠️  SIN MOVIMIENTOS CONFIGURADOS");
    return;
  }

  Serial.print("    🎯 EJECUTANDO SERVO ");
  Serial.print(servoID);
  Serial.print(" - ");
  Serial.print(servoConfigs[servoID].name);
  Serial.print(" | Duración: ");
  Serial.print(maxDuration);
  Serial.println("ms");

  // ⬅️ SIEMPRE USAR LA NUEVA FUNCIÓN MEJORADA
  executeServoMovementAdvanced(servoID, movement, maxDuration);
}


// 🔥 REEMPLAZA COMPLETAMENTE ESTA FUNCIÓN EN EL ARDUINO
void executeServoMovementAdvanced(uint8_t servoID, ServoMovement movement, uint16_t maxDuration) {
  if (servoID >= 32 || !servoConfigs[servoID].enabled || movement.stepCount == 0) {
    Serial.print("    ❌ SERVO ");
    Serial.print(servoID);
    Serial.println(" NO DISPONIBLE O SIN MOVIMIENTOS");
    return;
  }

  unsigned long startTime = millis();
  unsigned long elapsed = 0;
  int cycleCount = 0;
  
  Serial.print("      🎭 MOVIMIENTO AVANZADO Servo ");
  Serial.print(servoID);
  Serial.print(" - Pasos: ");
  Serial.print(movement.stepCount);
  Serial.print(" - Máximo: ");
  Serial.print(maxDuration);
  Serial.println("ms");

  // BUCLE PRINCIPAL - EJECUCIÓN REAL CON VELOCIDAD
  while (elapsed < maxDuration) {
    cycleCount++;
    
    Serial.print("      🔁 Ciclo ");
    Serial.print(cycleCount);
    Serial.print(" - Tiempo restante: ");
    Serial.print(maxDuration - elapsed);
    Serial.println("ms");
    
    int currentPosition = currentServoPositions[servoID];
    
    // EJECUTAR TODOS LOS PASOS
    for (int step = 0; step < movement.stepCount; step++) {
      elapsed = millis() - startTime;
      if (elapsed >= maxDuration) break;
      
      int targetAngle = movement.angles[step];
      uint16_t baseStepTime = movement.times[step]; // Tiempo base
      uint8_t velocity = movement.velocities[step]; // Velocidad (0-100%)
      uint16_t delayAfter = movement.delays[step];
      
      // 🚀 APLICAR VELOCIDAD REAL AL TIEMPO
      uint16_t actualStepTime = applyVelocityToTime(baseStepTime, velocity);
      
      Serial.print("        📈 Paso ");
      Serial.print(step + 1);
      Serial.print(": De ");
      Serial.print(currentPosition);
      Serial.print("° a ");
      Serial.print(targetAngle);
      Serial.print("° | TBase: ");
      Serial.print(baseStepTime);
      Serial.print("ms | Vel: ");
      Serial.print(velocity);
      Serial.print("% | TReal: ");
      Serial.print(actualStepTime);
      Serial.println("ms");
      
      // MOVER SERVO CON EL TIEMPO REAL AJUSTADO POR VELOCIDAD
      unsigned long stepStart = millis();
      unsigned long stepElapsed = 0;
      
      while (stepElapsed < actualStepTime && (millis() - startTime) < maxDuration) {
        stepElapsed = millis() - stepStart;
        elapsed = millis() - startTime;
        
        // Calcular progreso (0.0 a 1.0)
        float progress = (float)stepElapsed / (float)actualStepTime;
        
        // Aplicar curva suave
        progress = easeInOutCubic(progress);
        
        // Calcular ángulo intermedio
        int angleDifference = targetAngle - currentPosition;
        int intermediateAngle = currentPosition + (angleDifference * progress);
        
        // Limitar ángulo
        if (intermediateAngle < -90) intermediateAngle = -90;
        if (intermediateAngle > 90) intermediateAngle = 90;
        
        // Mover servo
        int pulse = angleToPulseINJORA(intermediateAngle);
        setServoPulse(servoID, pulse);
        
        delay(15); // Control más frecuente
      }
      
      // POSICIÓN FINAL EXACTA
      int finalPulse = angleToPulseINJORA(targetAngle);
      setServoPulse(servoID, finalPulse);
      currentServoPositions[servoID] = targetAngle;
      currentPosition = targetAngle;
      
      // DELAY DESPUÉS DEL MOVIMIENTO
      if (delayAfter > 0 && (millis() - startTime) < maxDuration) {
        Serial.print("        ⏸️  Delay: ");
        Serial.print(delayAfter);
        Serial.println("ms");
        
        unsigned long delayStart = millis();
        while ((millis() - delayStart) < delayAfter && (millis() - startTime) < maxDuration) {
          delay(10);
        }
      }
      
      elapsed = millis() - startTime;
      if (elapsed >= maxDuration) break;
    }
    
    elapsed = millis() - startTime;
  }
  
  Serial.print("    ✅ MOVIMIENTO COMPLETADO: ");
  Serial.print(cycleCount);
  Serial.print(" ciclos - ");
  Serial.print(elapsed);
  Serial.print("ms de ");
  Serial.print(maxDuration);
  Serial.println("ms totales");
}

// 🚀 NUEVA FUNCIÓN CRÍTICA - APLICA VELOCIDAD AL TIEMPO
uint16_t applyVelocityToTime(uint16_t baseTime, uint8_t velocity) {
  // Convertir velocidad 0-100% a factor de tiempo
  float velocityFactor;
  
  if (velocity <= 5) {
    velocityFactor = 4.0;      // Muy muy lento - 4x más tiempo
    Serial.print("      🐌🐌 VELOCIDAD EXTREMADAMENTE LENTA - ");
  } else if (velocity <= 15) {
    velocityFactor = 3.0;      // Muy lento - 3x más tiempo
    Serial.print("      🐌 VELOCIDAD MUY LENTA - ");
  } else if (velocity <= 30) {
    velocityFactor = 2.0;      // Lento - 2x más tiempo
    Serial.print("      🚶 VELOCIDAD LENTA - ");
  } else if (velocity <= 50) {
    velocityFactor = 1.5;      // Moderado - 1.5x más tiempo
    Serial.print("      🚗 VELOCIDAD MODERADA - ");
  } else if (velocity <= 70) {
    velocityFactor = 1.0;      // Normal - tiempo base
    Serial.print("      🚄 VELOCIDAD RÁPIDA - ");
  } else if (velocity <= 85) {
    velocityFactor = 0.7;      // Rápido - 0.7x del tiempo
    Serial.print("      🚀 VELOCIDAD MUY RÁPIDA - ");
  } else {
    velocityFactor = 0.4;      // Máxima velocidad - 0.4x del tiempo
    Serial.print("      ⚡ VELOCIDAD MÁXIMA - ");
  }
  
  uint16_t adjustedTime = baseTime * velocityFactor;
  
  // Límites de seguridad
  if (adjustedTime < 500) adjustedTime = 500;    // Mínimo 500ms
  if (adjustedTime > 30000) adjustedTime = 30000; // Máximo 30 segundos
  
  Serial.print("Factor: ");
  Serial.print(velocityFactor);
  Serial.print(" | Tiempo: ");
  Serial.print(baseTime);
  Serial.print("ms -> ");
  Serial.print(adjustedTime);
  Serial.println("ms");
  
  return adjustedTime;
}

// 🔧 FUNCIÓN MEJORADA - CONTROL DE VELOCIDAD REAL
void executeAdvancedServoControl(uint8_t servoID, int startAngle, int targetAngle, 
                                unsigned long elapsedTime, unsigned long totalTime, 
                                uint8_t velocity) {
  
  if (totalTime == 0 || velocity == 0) {
    // Movimiento instantáneo o velocidad 0
    int pulse = angleToPulseINJORA(targetAngle);
    setServoPulse(servoID, pulse);
    currentServoPositions[servoID] = targetAngle;
    return;
  }
  
  // Calcular progreso (0.0 a 1.0)
  float progress = (float)elapsedTime / (float)totalTime;
  
  // APLICAR CURVA DE VELOCIDAD
  float velocityFactor = (float)velocity / 100.0;
  
  // Curva suavizada - más lento al inicio y final
  if (velocityFactor < 1.0) {
    // Aplicar curva de ease-in-out para movimientos más suaves
    progress = easeInOutCubic(progress);
    
    // Reducir velocidad según el factor
    progress = progress * velocityFactor;
    
    // Limitar progreso
    if (progress > 1.0) progress = 1.0;
  }
  
  // Calcular ángulo intermedio
  int angleDifference = targetAngle - startAngle;
  int currentAngle = startAngle + (angleDifference * progress);
  
  // Mover servo
  int pulse = angleToPulseINJORA(currentAngle);
  setServoPulse(servoID, pulse);
  
  // Actualizar posición actual
  currentServoPositions[servoID] = currentAngle;
}

// 🎯 FUNCIÓN AUXILIAR PARA CURVA SUAVE
float easeInOutCubic(float t) {
  return t < 0.5 ? 4 * t * t * t : 1 - pow(-2 * t + 2, 3) / 2;
}

// ⚡ FUNCIÓN OPTIMIZADA PARA MOVER SERVOS
void setServoPulse(uint8_t servoID, int pulse) {
  if (servoID < 16 && pca1_detected) {
    pca1.setPWM(servoID, 0, pulse);
  } else if (servoID >= 16 && pca2_detected) {
    pca2.setPWM(servoID - 16, 0, pulse);
  }
}



void executeGroupInSequence(uint8_t groupID, uint8_t behavior, uint16_t totalDuration) {
  if (groupID >= 0 && groupID < programConfig.groupCount) {
    Group group = programConfig.groups[groupID];
    
    // 🟢 NUEVO: USAR REPRODUCCIÓN EXACTA
    Serial.print("    👥 GRUPO EXACTO: ");
    Serial.print(group.name);
    Serial.print(" | Servos: ");
    Serial.print(group.servoCount);
    Serial.print(" | Duración: ");
    Serial.print(totalDuration);
    Serial.println("ms - REPRODUCCIÓN EXACTA");
    
    // Buscar el ID del grupo en programConfig
    for (int i = 0; i < programConfig.groupCount; i++) {
      if (strcmp(programConfig.groups[i].name, group.name) == 0) {
        executeGroupExactSimultaneous(i); // ← EJECUCIÓN EXACTA
        return;
      }
    }
    
    // Fallback si no se encuentra
    Serial.println("      ⚠️  Grupo no encontrado, usando método legacy");
    executeGroupSequential(group, totalDuration);
  }
}


void testServoMovement(uint8_t servoID) {
  if (servoID >= 32 || !servoConfigs[servoID].enabled) {
    Serial.print("❌ Servo ");
    Serial.print(servoID);
    Serial.println(" no disponible");
    return;
  }

  Serial.print("🎬 TEST COMPLETO Servo ");
  Serial.print(servoID);
  Serial.print(" - ");
  Serial.println(servoConfigs[servoID].name);

  // Intentar usar los movimientos guardados (si existen)
  ServoMovement &mv = servoConfigs[servoID].movement;
  if (mv.stepCount > 0) {
    for (int i = 0; i < mv.stepCount; i++) {
      int angle = mv.angles[i];
      unsigned long t = mv.times[i];
      Serial.print("  ⇢ Paso "); Serial.print(i); Serial.print(" -> Angle "); Serial.print(angle); Serial.print(" in "); Serial.print(t); Serial.println(" ms");
      moveServoToAngle(servoID, angle, t);
      if (mv.delays[i] > 0) delay(mv.delays[i]);
    }
  } else {
    // Fallback: comportamiento de test legacy (tiempos constantes)
    Serial.println("  ➡️ Moviendo a 90° (lento)...");
    moveServoToAngle(servoID, 90, 4000); // 4 segundos
    delay(1000);

    Serial.println("  ⬅️ Moviendo a -90° (lento)...");
    moveServoToAngle(servoID, -90, 4000); // 4 segundos
    delay(1000);

    Serial.println("  ⏺️ Moviendo a 0° (normal)...");
    moveServoToAngle(servoID, 0, 2000); // 2 segundos
    delay(1000);
  }

  // APAGAR SERVO (mantener comportamiento original)
  if (servoID < 16 && pca1_detected) {
    pca1.setPWM(servoID, 0, 0);
  } else if (servoID >= 16 && pca2_detected) {
    pca2.setPWM(servoID - 16, 0, 0);
  }

  Serial.println("✅ TEST COMPLETADO");
}


// 🔧 FUNCIÓN AUXILIAR PARA MOVER A ÁNGULO ESPECÍFICO
void moveServoToAngle(uint8_t servoID, int targetAngle, unsigned long moveTime) {
  int startAngle = currentServoPositions[servoID];
  unsigned long startTime = millis();
  
  Serial.print("      🎯 Moviendo servo ");
  Serial.print(servoID);
  Serial.print(" de ");
  Serial.print(startAngle);
  Serial.print("° a ");
  Serial.print(targetAngle);
  Serial.print("° en ");
  Serial.print(moveTime);
  Serial.println("ms");
  
  while (millis() - startTime < moveTime) {
    unsigned long elapsed = millis() - startTime;
    float progress = (float)elapsed / (float)moveTime;
    
    // Aplicar curva suave
    progress = easeInOutCubic(progress);
    
    int currentAngle = startAngle + (targetAngle - startAngle) * progress;
    int pulse = angleToPulseINJORA(currentAngle);
    setServoPulse(servoID, pulse);
    
    delay(20);
  }
  
  // Posición final exacta
  int finalPulse = angleToPulseINJORA(targetAngle);
  setServoPulse(servoID, finalPulse);
  currentServoPositions[servoID] = targetAngle;
  
  Serial.println("      ✅ Movimiento completado");
}

void handleSaveSequence() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    
    Serial.println("💾 Guardando secuencia...");
    Serial.println("Datos recibidos:");
    Serial.println(body);
    
    // Resetear secuencia actual
    currentSequence.blockCount = 0;
    
    // AUMENTAR TAMAÑO DEL JSON DOCUMENT
    DynamicJsonDocument doc(12288);  // Cambiado de 4096 a 12288
    DeserializationError error = deserializeJson(doc, body);
    
    if (!error) {
      // Obtener nombre
      const char* nameStr = doc["sequenceName"];
      strncpy(currentSequence.sequenceName, nameStr ? nameStr : "Sin nombre", 19);
      
      // Obtener bloques (INCLUYENDO DURACIÓN CORRECTA)
      JsonArray blocks = doc["blocks"];
      for (JsonObject block : blocks) {
        if (currentSequence.blockCount >= 50) break;  // Usar nueva constante
        
        TimelineBlock newBlock;
        const char* typeStr = block["type"];
        
        // Tipo
        if (strcmp(typeStr, "servo") == 0) newBlock.type = 0;
        else if (strcmp(typeStr, "group") == 0) newBlock.type = 1;
        else if (strcmp(typeStr, "behavior") == 0) newBlock.type = 2;
        else if (strcmp(typeStr, "time") == 0) newBlock.type = 3;
        
        // Target ID
        newBlock.targetID = block["targetID"];
        
        // Nombre
        const char* blockName = block["name"];
        strncpy(newBlock.name, blockName ? blockName : "", 19);
        
        // ⬅️ CORRECCIÓN CRÍTICA: CAPTURAR DURACIÓN ACTUALIZADA
        newBlock.duration = block["duration"];
        
        // Comportamiento
        if (block.containsKey("behavior") && !block["behavior"].isNull()) {
          const char* behaviorStr = block["behavior"];
          if (strcmp(behaviorStr, "sequential") == 0) newBlock.behavior = 0;
          else if (strcmp(behaviorStr, "simultaneous") == 0) newBlock.behavior = 1;
          else if (strcmp(behaviorStr, "random") == 0) newBlock.behavior = 2;
        } else {
          newBlock.behavior = 0;
        }
        
        currentSequence.blocks[currentSequence.blockCount] = newBlock;
        currentSequence.blockCount++;
        
        Serial.print("  📦 Bloque ");
        Serial.print(currentSequence.blockCount);
        Serial.print(": ");
        Serial.print(newBlock.name);
        Serial.print(" - Duración: ");
        Serial.print(newBlock.duration);
        Serial.println("ms");
      }
      
      // Guardar en EEPROM
      EEPROM.put(SEQUENCE_EEPROM_OFFSET, currentSequence);
      EEPROM.commit();
      
      Serial.print("✅ Secuencia guardada: ");
      Serial.print(currentSequence.sequenceName);
      Serial.print(" - ");
      Serial.print(currentSequence.blockCount);
      Serial.println(" bloques");
      
      server.send(200, "application/json", "{\"status\":\"success\"}");
      return;
    } else {
      Serial.print("❌ Error parseando JSON: ");
      Serial.println(error.c_str());
    }
  }
  
  server.send(400, "application/json", "{\"status\":\"error\"}");
}



void handleLoadSequence() {
  
  String json = "{\"blocks\":[";
  
  for (int i = 0; i < currentSequence.blockCount; i++) {
    json += "{";
    json += "\"type\":\"";
    if (currentSequence.blocks[i].type == 0) json += "servo";
    else if (currentSequence.blocks[i].type == 1) json += "group";
    else if (currentSequence.blocks[i].type == 2) json += "behavior";
    else if (currentSequence.blocks[i].type == 3) json += "time"; // ⬅️ AGREGAR TIEMPO
    json += "\",\"targetID\":";
    json += String(currentSequence.blocks[i].targetID);
    json += ",\"name\":\"";
    json += String(currentSequence.blocks[i].name);
    json += "\",\"duration\":";
    json += String(currentSequence.blocks[i].duration);
    json += ",\"behavior\":";
    if (currentSequence.blocks[i].type == 2) {
      json += "\"";
      if (currentSequence.blocks[i].behavior == 0) json += "sequential";
      else if (currentSequence.blocks[i].behavior == 1) json += "simultaneous";
      else if (currentSequence.blocks[i].behavior == 2) json += "random";
      json += "\"";
    } else {
      json += "null";
    }
    json += "}";
    if (i < currentSequence.blockCount - 1) json += ",";
  }
  
  json += "],\"sequenceName\":\"";
  json += String(currentSequence.sequenceName);
  json += "\",\"blockCount\":";
  json += String(currentSequence.blockCount);
  json += "}";
  
  server.send(200, "application/json", json);
}



void setupPCA9685() {
  Serial.println("Inicializando PCA9685...");
  
  // INICIALIZAR PRIMERO LA PLACA #2 (la problemática)
  pca2_detected = detectPCA9685(0x41);
  if (pca2_detected) {
    pca2.begin();
    pca2.setPWMFreq(50);
    Serial.println("PCA9685 #2 encontrado en dirección 0x41");
  }
  
  // LUEGO LA PLACA #1 (la que funciona)
  pca1_detected = detectPCA9685(0x40);
  if (pca1_detected) {
    pca1.begin();
    pca1.setPWMFreq(50);
    Serial.println("PCA9685 #1 encontrado en dirección 0x40");
  }
}


// === FUNCIÓN PARA INICIAR EJECUCIÓN EN LOOP ===
void startLoopExecution() {
  systemRunning = true;
  loopEnabled = true;
  programStartTime = millis();
  
  Serial.println("🔄 EJECUCIÓN EN LOOP INICIADA");
  Serial.print("Modo: ");
  Serial.println(execConfig.currentMode);
  Serial.print("Intervalo: ");
  Serial.print(execConfig.loopInterval);
  Serial.println("ms");
  Serial.print("Sensor PIR: ");
  Serial.println(execConfig.useSensor ? "ACTIVADO" : "DESACTIVADO");
}

// === FUNCIÓN PARA DETENER EJECUCIÓN ===
void stopLoopExecution() {
  systemRunning = false;
  loopEnabled = false;
  sequenceMode = false;  
  isExecutingSequence = false; 
  
  // Mover todos los servos a posición central y apagar
  returnToCenterAndOff();
  
  Serial.println("⏹️ EJECUCIÓN DETENIDA");
}

void handleRedetect() {
  Serial.println("Forzando redetección de PCA9685...");
  setupPCA9685();
  
  String json = "{";
  json += "\"pca1\":{\"detected\":";
  json += pca1_detected ? "true" : "false";
  json += "},";
  json += "\"pca2\":{\"detected\":";
  json += pca2_detected ? "true" : "false";
  json += "}}";
  
  server.send(200, "application/json", json);
}


// Manejar página de grupos
void handleGroups() {
  server.send(200, "text/html", GROUPS_HTML);
}



void initEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  
  Serial.println("=== 🔄 INICIALIZANDO EEPROM ===");
  Serial.print("Tamaño EEPROM: ");
  Serial.print(EEPROM_SIZE);
  Serial.println(" bytes");
  Serial.print("Tamaño ServoConfig: ");
  Serial.println(sizeof(ServoConfig));
  
  
  
  // CARGAR CONFIGURACIÓN DE SERVOS CON VALIDACIÓN
  Serial.println("📥 Cargando configuración de 32 servos...");
  
  int servosCargados = 0;
  int servosConMovimientos = 0;
  
  for (int i = 0; i < 32; i++) {
    EEPROM.get(i * sizeof(ServoConfig), servoConfigs[i]);
    
    // ⬅️ VALIDACIÓN Y CORRECCIÓN DE DATOS CORRUPTOS
    if (servoConfigs[i].movement.stepCount > 6) {
      Serial.print("  🔧 Reparando servo ");
      Serial.print(i);
      Serial.println(" - stepCount inválido");
      servoConfigs[i].movement.stepCount = 0;
    }
    
    // VALIDAR Y CORREGIR NOMBRE
    if (strlen(servoConfigs[i].name) == 0 || strlen(servoConfigs[i].name) > 19) {
      sprintf(servoConfigs[i].name, "Servo %d", i);
    }
    
    // ⬅️ INICIALIZACIÓN SEGURA DE NUEVOS PARÁMETROS
    for (int j = 0; j < 6; j++) {
      // VALIDAR ÁNGULOS (-90 a 90)
      if (servoConfigs[i].movement.angles[j] < -90 || servoConfigs[i].movement.angles[j] > 90) {
        servoConfigs[i].movement.angles[j] = 0;
      }
      
      // VALIDAR TIEMPOS (100-30000ms)
      if (servoConfigs[i].movement.times[j] < 100 || servoConfigs[i].movement.times[j] > 30000) {
        servoConfigs[i].movement.times[j] = 1000;
      }
      
      // ⬅️ INICIALIZAR VELOCIDADES SI NO EXISTEN
      if (servoConfigs[i].movement.velocities[j] > 100) {
        servoConfigs[i].movement.velocities[j] = 50; // 50% por defecto
      }
      
      // ⬅️ INICIALIZAR DELAYS SI NO EXISTEN
      if (servoConfigs[i].movement.delays[j] > 10000) {
        servoConfigs[i].movement.delays[j] = 0; // 0ms por defecto
      }
      
      // ⬅️ INICIALIZAR ACELERACIONES SI NO EXISTEN
      if (servoConfigs[i].movement.accelerations[j] > 100) {
        servoConfigs[i].movement.accelerations[j] = 50; // 50% por defecto
      }
    }
    
    if (servoConfigs[i].enabled) {
      servosCargados++;
      if (servoConfigs[i].movement.stepCount > 0) {
        servosConMovimientos++;
      }
    }
  }
  
  Serial.print("✅ Servos cargados: ");
  Serial.print(servosCargados);
  Serial.print(" habilitados, ");
  Serial.print(servosConMovimientos);
  Serial.println(" con movimientos");

  // CARGAR CONFIGURACIÓN DE PROGRAMA CON VALIDACIÓN
  Serial.println("📥 Cargando configuración de programa...");
  EEPROM.get(PROGRAM_EEPROM_OFFSET, programConfig);
  
  // VALIDAR programConfig
  if (programConfig.groupCount > 6) {
    Serial.println("⚠️  Datos de ProgramConfig inválidos - Inicializando defaults");
    programConfig.groupCount = 0;
    programConfig.executionMode = 0;
    programConfig.groupBehavior = 0;
    
    for (int i = 0; i < 6; i++) {
      programConfig.groups[i].name[0] = '\0';
      programConfig.groups[i].enabled = false;
      programConfig.groups[i].servoCount = 0;
      for (int j = 0; j < 8; j++) {
        programConfig.groups[i].servos[j] = 0;
      }
    }
    
    EEPROM.put(PROGRAM_EEPROM_OFFSET, programConfig);
    EEPROM.commit();
  }
  
  Serial.print("✅ Grupos cargados: ");
  Serial.println(programConfig.groupCount);

  // CARGAR CONFIGURACIÓN DE EJECUCIÓN CON VALIDACIÓN
  Serial.println("📥 Cargando configuración de ejecución...");
  EEPROM.get(EXEC_CONFIG_EEPROM_OFFSET, execConfig);
  
  // Si es la primera vez o datos inválidos, usar valores por defecto
  if (execConfig.loopInterval == 0 || execConfig.loopInterval > 60000 || execConfig.loopInterval == 0xFFFF) {
    Serial.println("⚠️  Configuración de ejecución inválida - Inicializando defaults");
    execConfig.loopInterval = 5000;
    execConfig.useSensor = false;
    execConfig.currentMode = 0;
    EEPROM.put(EXEC_CONFIG_EEPROM_OFFSET, execConfig);
    EEPROM.commit();
  }
  
  Serial.print("✅ Config ejecución - Intervalo: ");
  Serial.print(execConfig.loopInterval);
  Serial.print("ms, Sensor: ");
  Serial.println(execConfig.useSensor ? "ACTIVADO" : "DESACTIVADO");

  // CARGAR SECUENCIA CON VALIDACIÓN
  Serial.println("📥 Cargando secuencia...");
  EEPROM.get(SEQUENCE_EEPROM_OFFSET, currentSequence);
  
  if (currentSequence.blockCount > MAX_BLOCKS || currentSequence.blockCount == 0xFF) {
    Serial.println("⚠️  Secuencia inválida - Inicializando defaults");
    currentSequence.blockCount = 0;
    strncpy(currentSequence.sequenceName, "Secuencia Inicial", 19);
    EEPROM.put(SEQUENCE_EEPROM_OFFSET, currentSequence);
    EEPROM.commit();
  }
  
  Serial.print("✅ Secuencia cargada: '");
  Serial.print(currentSequence.sequenceName);
  Serial.print("' - ");
  Serial.print(currentSequence.blockCount);
  Serial.println(" bloques");

  // VERIFICACIÓN FINAL DE MEMORIA
  Serial.println("=== 📊 VERIFICACIÓN DE MEMORIA ===");
  Serial.print("Espacio usado por 32 servos: ");
  Serial.print(32 * sizeof(ServoConfig));
  Serial.println(" bytes");
  
  Serial.print("Espacio total EEPROM: ");
  Serial.print(EEPROM_SIZE);
  Serial.println(" bytes");
  
  Serial.print("Espacio libre estimado: ");
  Serial.print(EEPROM_SIZE - (32 * sizeof(ServoConfig) + sizeof(ProgramConfig) + sizeof(ExecutionConfig) + sizeof(Sequence)));
  Serial.println(" bytes");

  // INICIALIZAR VARIABLES DE EJECUCIÓN EN MEMORIA (NO EEPROM)
  systemRunning = false;
  sequenceExecution = false;
  sequenceMode = false;
  loopEnabled = false;
  pirTriggered = false;
  currentStep = 0;
  totalSteps = 0;
  isExecutingSequence = false;
  currentExecutionStep = 0;
  executionStartTime = 0;
  programStartTime = 0;
  lastLoopTime = 0;
  lastPIRTrigger = 0;
  lastPIRTime = 0;
  currentExecutionInfo = "";

  // ⬅️ NUEVO: VERIFICAR INTEGRIDAD DE DATOS AVANZADOS
  Serial.println("=== 🔍 VERIFICANDO PARÁMETROS AVANZADOS ===");
  int servosConParametrosAvanzados = 0;
  
  for (int i = 0; i < 32; i++) {
    if (servoConfigs[i].enabled && servoConfigs[i].movement.stepCount > 0) {
      bool tieneAvanzados = false;
      
      for (int j = 0; j < servoConfigs[i].movement.stepCount; j++) {
        if (servoConfigs[i].movement.velocities[j] != 50 || 
            servoConfigs[i].movement.accelerations[j] != 50 || 
            servoConfigs[i].movement.delays[j] != 0) {
          tieneAvanzados = true;
          break;
        }
      }
      
      if (tieneAvanzados) {
        servosConParametrosAvanzados++;
        Serial.print("  🚀 Servo ");
        Serial.print(i);
        Serial.print(" - ");
        Serial.print(servoConfigs[i].name);
        Serial.println(" tiene parámetros avanzados");
      }
    }
  }
  
  Serial.print("✅ Servos con parámetros avanzados: ");
  Serial.println(servosConParametrosAvanzados);

  Serial.println("=== ✅ EEPROM INICIALIZADA CORRECTAMENTE ===");
  
  // DEBUG OPCIONAL: MOSTRAR ESTADO DE ALGUNOS SERVOS
  #ifdef DEBUG_EEPROM
  Serial.println("=== 🐛 DEBUG EEPROM ===");
  for (int i = 0; i < min(5, 32); i++) { // Mostrar solo primeros 5 servos
    Serial.print("Servo ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(servoConfigs[i].name);
    Serial.print(" - ");
    Serial.print(servoConfigs[i].enabled ? "HABILITADO" : "DESHABILITADO");
    Serial.print(" - Pasos: ");
    Serial.println(servoConfigs[i].movement.stepCount);
    
    if (servoConfigs[i].movement.stepCount > 0) {
      Serial.print("  Parámetros avanzados: ");
      bool tieneAvanzados = false;
      for (int j = 0; j < servoConfigs[i].movement.stepCount; j++) {
        if (servoConfigs[i].movement.velocities[j] != 50 || 
            servoConfigs[i].movement.accelerations[j] != 50 || 
            servoConfigs[i].movement.delays[j] != 0) {
          tieneAvanzados = true;
          break;
        }
      }
      Serial.println(tieneAvanzados ? "SI" : "NO");
    }
  }
  Serial.println("=====================");
  #endif
}

void loadServoConfig() {
  for (int i = 0; i < 32; i++) {
    EEPROM.get(i * sizeof(ServoConfig), servoConfigs[i]);
    Serial.print("Servo ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(servoConfigs[i].name);
    Serial.print(" - ");
    Serial.println(servoConfigs[i].enabled ? "HABILITADO" : "DESHABILITADO");
  }
}

void saveServoConfig(int servoIndex, const char* name, bool enabled) {
  if (servoIndex >= 0 && servoIndex < 32) {
    strncpy(servoConfigs[servoIndex].name, name, 19);
    servoConfigs[servoIndex].name[19] = '\0';
    servoConfigs[servoIndex].enabled = enabled;
    
    EEPROM.put(servoIndex * sizeof(ServoConfig), servoConfigs[servoIndex]);
    EEPROM.commit();
    
    Serial.print("Servo ");
    Serial.print(servoIndex);
    Serial.print(" guardado: ");
    Serial.print(servoConfigs[servoIndex].name);
    Serial.print(" - ");
    Serial.println(servoConfigs[servoIndex].enabled ? "HABILITADO" : "DESHABILITADO");
  }
}

void handleRoot() {
  server.send(200, "text/html", MAIN_HTML);
}

void handleSetup() {
  server.send(200, "text/html", SETUP_HTML);
}

void handleProgram() {
  server.send(200, "text/html", PROGRAM_HTML);
}

void handleDetect() {
  Serial.print("Estado actual - PCA1: ");
  Serial.print(pca1_detected);
  Serial.print(" PCA2: ");
  Serial.println(pca2_detected);
  
  String json = "{";
  json += "\"pca1\":{\"detected\":";
  json += pca1_detected ? "true" : "false";
  json += "},";
  json += "\"pca2\":{\"detected\":";
  json += pca2_detected ? "true" : "false";
  json += "}}";
  
  server.send(200, "application/json", json);
}

void handleGetConfig() {
  String json = "[";
  for (int i = 0; i < 32; i++) {
    json += "{\"name\":\"";
    json += servoConfigs[i].name;
    json += "\",\"enabled\":";
    json += servoConfigs[i].enabled ? "true" : "false";
    json += "}";
    if (i < 31) json += ",";
  }
  json += "]";
  
  server.send(200, "application/json", json);
}

void handleSaveName() {
  if (server.hasArg("servo") && server.hasArg("name") && server.hasArg("enabled")) {
    int servo = server.arg("servo").toInt();
    String name = server.arg("name");
    bool enabled = server.arg("enabled") == "true";
    
    saveServoConfig(servo, name.c_str(), enabled);
    server.send(200, "application/json", "{\"status\":\"success\"}");
  } else {
    server.send(400, "application/json", "{\"status\":\"error\"}");
  }
}

void handleSaveSetup() {
  server.send(200, "application/json", "{\"status\":\"success\"}");
}
void handleServoSweep() {
  if (server.hasArg("servo")) {
    int servo = server.arg("servo").toInt();
    
    // Valores CORREGIDOS para INJORA
    int center = angleToPulseINJORA(0);    // Posición central
    int left = angleToPulseINJORA(-20);    // 20 grados a la izquierda
    int right = angleToPulseINJORA(20);    // 20 grados a la derecha
    
    if (servo >= 0 && servo < 16 && pca1_detected) {
      pca1.setPWM(servo, 0, center);
      delay(500);
      pca1.setPWM(servo, 0, left);
      delay(500);
      pca1.setPWM(servo, 0, right);
      delay(500);
      pca1.setPWM(servo, 0, center);
      delay(500);
      pca1.setPWM(servo, 0, 0);
    } else if (servo >= 16 && servo < 32 && pca2_detected) {
      pca2.setPWM(servo - 16, 0, center);
      delay(500);
      pca2.setPWM(servo - 16, 0, left);
      delay(500);
      pca2.setPWM(servo - 16, 0, right);
      delay(500);
      pca2.setPWM(servo - 16, 0, center);
      delay(500);
      pca2.setPWM(servo - 16, 0, 0);
    }
    
    server.send(200, "application/json", "{\"status\":\"success\"}");
  }
}

// Obtener movimientos de todos los servos
void handleGetMovements() {
  String json = "[";
  for (int i = 0; i < 32; i++) {
    json += "{";
    json += "\"servoIndex\":" + String(i) + ",";
    json += "\"name\":\"" + String(servoConfigs[i].name) + "\",";
    json += "\"enabled\":" + String(servoConfigs[i].enabled ? "true" : "false") + ",";
    json += "\"stepCount\":" + String(servoConfigs[i].movement.stepCount) + ",";
    
    // Ángulos
    json += "\"angles\":[";
    for (int j = 0; j < 6; j++) {
      json += String(servoConfigs[i].movement.angles[j]);
      if (j < 5) json += ",";
    }
    json += "],";
    
    // Tiempos
    json += "\"times\":[";
    for (int j = 0; j < 6; j++) {
      json += String(servoConfigs[i].movement.times[j]);
      if (j < 5) json += ",";
    }
    json += "],";
    
    // ⬅️ NUEVO: Velocidades
    json += "\"velocities\":[";
    for (int j = 0; j < 6; j++) {
      json += String(servoConfigs[i].movement.velocities[j]);
      if (j < 5) json += ",";
    }
    json += "],";
    
    // ⬅️ NUEVO: Delays
    json += "\"delays\":[";
    for (int j = 0; j < 6; j++) {
      json += String(servoConfigs[i].movement.delays[j]);
      if (j < 5) json += ",";
    }
    json += "],";
    
    // ⬅️ NUEVO: Aceleraciones
    json += "\"accelerations\":[";
    for (int j = 0; j < 6; j++) {
      json += String(servoConfigs[i].movement.accelerations[j]);
      if (j < 5) json += ",";
    }
    json += "]";
    
    json += "}";
    if (i < 31) json += ",";
  }
  json += "]";
  
  server.send(200, "application/json", json);
}

void handleResetEEPROM() {
  // Resetear EEPROM
  for (int i = 0; i < 32; i++) {
    sprintf(servoConfigs[i].name, "Servo %d", i);
    servoConfigs[i].enabled = false;
    servoConfigs[i].movement.stepCount = 0;
    for (int j = 0; j < 6; j++) {
      servoConfigs[i].movement.angles[j] = 0;
      servoConfigs[i].movement.times[j] = 1000;
      servoConfigs[i].movement.delays[j] = 0;
      servoConfigs[i].movement.velocities[j] = 50;  // ⬅️ CORREGIDO: velocities en lugar de speeds
      servoConfigs[i].movement.accelerations[j] = 50;
    }
    EEPROM.put(i * sizeof(ServoConfig), servoConfigs[i]);
  }
  EEPROM.commit();
  
  Serial.println("EEPROM reseteada desde interfaz web");
  server.send(200, "application/json", "{\"status\":\"success\"}");
}

// Guardar movimiento de un servo

void handleSaveMovement() {
  if (server.hasArg("servo") && server.hasArg("stepCount") && 
      server.hasArg("angles") && server.hasArg("times") &&
      server.hasArg("velocities") && server.hasArg("delays") && server.hasArg("accelerations")) {
    
    int servoIndex = server.arg("servo").toInt();
    
    // VALIDACIÓN CRÍTICA PARA 32 SERVOS
    if (servoIndex < 0 || servoIndex >= 32) {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Índice de servo inválido\"}");
      return;
    }
    
    int stepCount = server.arg("stepCount").toInt();
    
    // Validar stepCount
    if (stepCount < 0 || stepCount > 6) {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Número de pasos inválido\"}");
      return;
    }
    
    // LÍMITES MÁS SEGUROS
    const int MAX_DELAY = 5000;  // 5 segundos máximo
    const int MAX_TIME = 20000;  // 20 segundos máximo por paso
    
    String anglesStr = server.arg("angles");
    String timesStr = server.arg("times");
    String velocitiesStr = server.arg("velocities");
    String delaysStr = server.arg("delays");
    String accelerationsStr = server.arg("accelerations");
    
    Serial.print("💾 Guardando movimiento avanzado para servo ");
    Serial.print(servoIndex);
    Serial.print(" - Pasos: ");
    Serial.println(stepCount);
    
    // INICIALIZAR ARRAYS CON VALORES SEGUROS
    for (int i = 0; i < 6; i++) {
      servoConfigs[servoIndex].movement.angles[i] = 0;
      servoConfigs[servoIndex].movement.times[i] = 1000;
      servoConfigs[servoIndex].movement.delays[i] = 0;
      servoConfigs[servoIndex].movement.velocities[i] = 50;
      servoConfigs[servoIndex].movement.accelerations[i] = 50;
    }
    
    // PARSEAR Y VALIDAR ÁNGULOS (-90 a 90)
    int angleIndex = 0;
    int startPos = 0;
    for (int i = 0; i < anglesStr.length() && angleIndex < stepCount; i++) {
      if (anglesStr.charAt(i) == ',' || i == anglesStr.length() - 1) {
        int endPos = (i == anglesStr.length() - 1) ? i + 1 : i;
        String angleStr = anglesStr.substring(startPos, endPos);
        int angle = angleStr.toInt();
        // Validar rango de ángulo
        if (angle < -90) angle = -90;
        if (angle > 90) angle = 90;
        servoConfigs[servoIndex].movement.angles[angleIndex] = angle;
        startPos = i + 1;
        angleIndex++;
      }
    }
    
    // PARSEAR TIEMPOS (100-20000ms)
    int timeIndex = 0;
    startPos = 0;
    for (int i = 0; i < timesStr.length() && timeIndex < stepCount; i++) {
      if (timesStr.charAt(i) == ',' || i == timesStr.length() - 1) {
        int endPos = (i == timesStr.length() - 1) ? i + 1 : i;
        String timeStr = timesStr.substring(startPos, endPos);
        int time = timeStr.toInt();
        // Validar rango de tiempo
        if (time < 100) time = 100;
        if (time > MAX_TIME) time = MAX_TIME;
        servoConfigs[servoIndex].movement.times[timeIndex] = time;
        startPos = i + 1;
        timeIndex++;
      }
    }
    
    // PARSEAR VELOCIDADES (0-100%)
    int velocityIndex = 0;
    startPos = 0;
    for (int i = 0; i < velocitiesStr.length() && velocityIndex < stepCount; i++) {
      if (velocitiesStr.charAt(i) == ',' || i == velocitiesStr.length() - 1) {
        int endPos = (i == velocitiesStr.length() - 1) ? i + 1 : i;
        String velocityStr = velocitiesStr.substring(startPos, endPos);
        int velocity = velocityStr.toInt();
        // Validar rango de velocidad
        if (velocity < 0) velocity = 0;
        if (velocity > 100) velocity = 100;
        servoConfigs[servoIndex].movement.velocities[velocityIndex] = velocity;
        startPos = i + 1;
        velocityIndex++;
      }
    }
    
    // PARSEAR DELAYS (0-5000ms)
    int delayIndex = 0;
    startPos = 0;
    for (int i = 0; i < delaysStr.length() && delayIndex < stepCount; i++) {
      if (delaysStr.charAt(i) == ',' || i == delaysStr.length() - 1) {
        int endPos = (i == delaysStr.length() - 1) ? i + 1 : i;
        String delayStr = delaysStr.substring(startPos, endPos);
        int delay = delayStr.toInt();
        // Validar rango de delay
        if (delay < 0) delay = 0;
        if (delay > MAX_DELAY) delay = MAX_DELAY;
        servoConfigs[servoIndex].movement.delays[delayIndex] = delay;
        startPos = i + 1;
        delayIndex++;
      }
    }
    
    // PARSEAR ACELERACIONES (0-100%)
    int accelerationIndex = 0;
    startPos = 0;
    for (int i = 0; i < accelerationsStr.length() && accelerationIndex < stepCount; i++) {
      if (accelerationsStr.charAt(i) == ',' || i == accelerationsStr.length() - 1) {
        int endPos = (i == accelerationsStr.length() - 1) ? i + 1 : i;
        String accelerationStr = accelerationsStr.substring(startPos, endPos);
        int acceleration = accelerationStr.toInt();
        // Validar rango de aceleración
        if (acceleration < 0) acceleration = 0;
        if (acceleration > 100) acceleration = 100;
        servoConfigs[servoIndex].movement.accelerations[accelerationIndex] = acceleration;
        startPos = i + 1;
        accelerationIndex++;
      }
    }
    
    // GUARDAR STEP COUNT
    servoConfigs[servoIndex].movement.stepCount = stepCount;
    
    // GUARDAR EN EEPROM
    EEPROM.put(servoIndex * sizeof(ServoConfig), servoConfigs[servoIndex]);
    if (EEPROM.commit()) {
      Serial.println("✅ Movimientos avanzados guardados correctamente");
      server.send(200, "application/json", "{\"status\":\"success\"}");
    } else {
      Serial.println("❌ Error guardando en EEPROM");
      server.send(500, "application/json", "{\"status\":\"error\",\"message\":\"Error EEPROM\"}");
    }
    
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Faltan parámetros\"}");
  }
}

// 🔧 FUNCIÓN AUXILIAR PARA PARSEAR Y VALIDAR ARRAYS



void applyServoSpeed(uint8_t servoID, int targetAngle, uint8_t speed, uint16_t stepTime) {

  if (servoID >= 32) return;
  if (speed < 1) speed = 1;
  if (speed > 10) speed = 10;

  int currentAngle = currentServoPositions[servoID];

  Serial.print(" 🚀 Velocidad ");
  Serial.print(speed);
  Serial.print("/10 - De ");
  Serial.print(currentAngle);
  Serial.print("° a ");
  Serial.print(targetAngle);
  Serial.print("° en ");
  Serial.print(stepTime);
  Serial.println("ms");

  if (currentAngle == targetAngle) {
    Serial.println(" ✅ Ya está en posición objetivo");
    delay(stepTime);
    return;
  }

  int currentPulse = angleToPulseINJORA(currentAngle);
  int targetPulse = angleToPulseINJORA(targetAngle);

  // VELOCIDAD 10 → directo
  if (speed == 10) {
    Serial.println(" ⚡⚡⚡ MOVIMIENTO DIRECTO");
    if (servoID < 16 && pca1_detected)
      pca1.setPWM(servoID, 0, targetPulse);
    else if (servoID >= 16 && pca2_detected)
      pca2.setPWM(servoID - 16, 0, targetPulse);
    currentServoPositions[servoID] = targetAngle;
    delay(stepTime);
    return;
  }

  // DEFINIR PASOS POR VELOCIDAD - PROGRESIÓN MÁS GRADUAL
  int steps = 0;
  switch(speed) {
    case 1: steps = 200; break;  // MUY MUY LENTO
    case 2: steps = 120; break;  // MUY LENTO
    case 3: steps = 80; break;   // LENTO
    case 4: steps = 60; break;   // MODERADO-LENTO
    case 5: steps = 40; break;   // MODERADO
    case 6: steps = 25; break;   // NORMAL-RÁPIDO
    case 7: steps = 15; break;   // RÁPIDO
    case 8: steps = 8; break;    // MUY RÁPIDO
    case 9: steps = 3; break;    // ULTRA RÁPIDO
  }

  int timePerStep = stepTime / steps;
  if (timePerStep < 5) timePerStep = 5;

  Serial.print(" 🔄 Movimiento ");
  Serial.print(steps);
  Serial.print(" pasos de ");
  Serial.print(timePerStep);
  Serial.println("ms");

  int pulseIncrement = (targetPulse - currentPulse) / steps;
  int angleIncrement = (targetAngle - currentAngle) / steps;

  for (int i = 1; i <= steps; i++) {
    int pulso = currentPulse + pulseIncrement * i;
    int intermediateAngle = currentAngle + angleIncrement * i;

    if (servoID < 16 && pca1_detected)
      pca1.setPWM(servoID, 0, pulso);
    else if (servoID >= 16 && pca2_detected)
      pca2.setPWM(servoID - 16, 0, pulso);

    currentServoPositions[servoID] = intermediateAngle;
    delay(timePerStep);
  }

  // Posición final exacta
  if (servoID < 16 && pca1_detected)
    pca1.setPWM(servoID, 0, targetPulse);
  else if (servoID >= 16 && pca2_detected)
    pca2.setPWM(servoID - 16, 0, targetPulse);

  currentServoPositions[servoID] = targetAngle;
}


void printSystemInfo() {
  Serial.println("=== INFORMACIÓN DEL SISTEMA ===");
  Serial.print("Chip: ");
  Serial.println(ESP.getChipModel());
  Serial.print("CPU Freq: ");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.println(" MHz");
  Serial.print("Flash Size: ");
  Serial.print(ESP.getFlashChipSize() / (1024 * 1024));
  Serial.println(" MB");
  Serial.print("Free Heap: ");
  Serial.print(ESP.getFreeHeap() / 1024);
  Serial.println(" KB");
  Serial.print("SDK Version: ");
  Serial.println(ESP.getSdkVersion());
  Serial.println("===============================");
}


// Probar movimiento de un servo
// Probar movimiento de un servo individual - SECUENCIA COMPLETA

void handleTestMovement() {
  if (server.hasArg("servo")) {
    int servoIndex = server.arg("servo").toInt();
    
    if (servoIndex < 0 || servoIndex >= 32) {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Servo inválido\"}");
      return;
    }
    
    server.send(200, "application/json", "{\"status\":\"success\"}");
    
    if (servoConfigs[servoIndex].enabled) {
      // USAR NUEVA FUNCIÓN DE TEST
      testServoMovement(servoIndex);
    } else {
      Serial.print("❌ Servo ");
      Serial.print(servoIndex);
      Serial.println(" no habilitado");
    }
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Falta parámetro servo\"}");
  }
}




void handleControl() {
  server.send(200, "text/html", CONTROL_HTML);
}

void handleGetControlStatus() {
  String json = "{";
  json += "\"running\":";
  json += systemRunning ? "true" : "false";
  
  // CORREGIR: Calcular runTime correctamente
  if (systemRunning) {
    json += ",\"runTime\":";
    json += String(millis() - programStartTime);
  } else {
    json += ",\"runTime\":0";
  }
  
  json += ",\"currentMode\":";
  json += String(execConfig.currentMode);
  json += ",\"loopInterval\":";
  json += String(execConfig.loopInterval);
  json += ",\"useSensor\":";
  json += execConfig.useSensor ? "true" : "false";
  
  // INFORMACIÓN DE PROGRESO
  json += ",\"currentStep\":";
  json += String(currentStep);
  json += ",\"totalSteps\":";
  json += String(totalSteps);
  json += ",\"sequenceName\":\"";
  json += executionSequence.sequenceName;
  json += "\"";
  
  json += "}";
  
  server.send(200, "application/json", json);
}



void handleGetGroups() {
  Serial.print("Enviando grupos - Count: ");
  Serial.println(programConfig.groupCount);
  
  String json = "{\"groups\":[";
  for (int i = 0; i < programConfig.groupCount; i++) {
    json += "{";
    json += "\"id\":" + String(i) + ",";
    json += "\"name\":\"" + String(programConfig.groups[i].name) + "\",";
    json += "\"enabled\":" + String(programConfig.groups[i].enabled ? "true" : "false") + ",";
    json += "\"servoCount\":" + String(programConfig.groups[i].servoCount) + ",";
    
    json += "\"servos\":[";
    for (int j = 0; j < programConfig.groups[i].servoCount; j++) {
      json += String(programConfig.groups[i].servos[j]);
      if (j < programConfig.groups[i].servoCount - 1) json += ",";
    }
    json += "]";
    
    json += "}";
    if (i < programConfig.groupCount - 1) json += ",";
  }
  json += "],\"groupCount\":" + String(programConfig.groupCount) + "}";
  
  Serial.print("JSON enviado: ");
  Serial.println(json);
  
  server.send(200, "application/json", json);
}


void handleTestGroup() {
  if (server.hasArg("id")) {
    int groupId = server.arg("id").toInt();
    
    if (groupId >= 0 && groupId < programConfig.groupCount) {
      Group group = programConfig.groups[groupId];
      
      Serial.print("🔧 TEST SIMULTÁNEO DE GRUPO: ");
      Serial.println(group.name);
      Serial.print("👥 Servos en el grupo: ");
      Serial.println(group.servoCount);
      
      // CALCULAR EL TIEMPO MÁXIMO NECESARIO PARA TODOS LOS SERVOS
      uint16_t maxDuration = 0;
      for (int i = 0; i < group.servoCount; i++) {
        int servoID = group.servos[i];
        if (servoID < 32 && servoConfigs[servoID].enabled) {
          ServoMovement movement = servoConfigs[servoID].movement;
          uint16_t totalTime = 0;
          for (int s = 0; s < movement.stepCount; s++) {
            totalTime += movement.times[s];
          }
          if (totalTime > maxDuration) {
            maxDuration = totalTime;
          }
          
          Serial.print("  🎯 Servo ");
          Serial.print(servoID);
          Serial.print(" - ");
          Serial.print(servoConfigs[servoID].name);
          Serial.print(" | Pasos: ");
          Serial.print(movement.stepCount);
          Serial.print(" | Tiempo total: ");
          Serial.print(totalTime);
          Serial.println("ms");
        }
      }
      
      // AGREGAR MARGEN DE SEGURIDAD
      maxDuration += 1000;
      Serial.print("⏱️ Duración simultánea: ");
      Serial.print(maxDuration);
      Serial.println("ms");
      
      // CREAR SECUENCIA TEMPORAL PARA EJECUCIÓN SIMULTÁNEA
      Sequence tempSequence;
      tempSequence.blockCount = group.servoCount;
      
      for (int i = 0; i < group.servoCount; i++) {
        int servoID = group.servos[i];
        
        TimelineBlock block;
        block.type = 0; // Servo individual
        block.targetID = servoID;
        strncpy(block.name, servoConfigs[servoID].name, 19);
        block.duration = maxDuration; // MISMA DURACIÓN PARA TODOS
        block.behavior = 1; // COMPORTAMIENTO SIMULTÁNEO
        
        tempSequence.blocks[i] = block;
      }
      
      // EJECUTAR EN SIMULTÁNEO
      Serial.println("🔄 INICIANDO EJECUCIÓN SIMULTÁNEA");
      executeSequenceTest(tempSequence);
      
      Serial.println("✅ TEST SIMULTÁNEO DE GRUPO COMPLETADO");
      server.send(200, "application/json", "{\"status\":\"success\"}");
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Grupo no encontrado\"}");
    }
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Falta parámetro id\"}");
  }
}

// === NUEVAS FUNCIONES PARA REPRODUCCIÓN EXACTA EN GRUPOS ===

// 🎯 FUNCIÓN CORREGIDA: Ejecutar grupo EXACTO como en Programación
void executeGroupExactSimultaneous(uint8_t groupID) {
  if (groupID >= programConfig.groupCount) {
    Serial.println("❌ Grupo no encontrado");
    return;
  }
  
  Group group = programConfig.groups[groupID];
  Serial.print("🎯 EJECUTANDO GRUPO EXACTO: ");
  Serial.print(group.name);
  Serial.print(" | Servos: ");
  Serial.println(group.servoCount);

  // 🟢 EJECUCIÓN DIRECTA: Cada servo reproduce su movimiento EXACTO en paralelo
  unsigned long startTime = millis();
  
  // Crear array para controlar cada servo
  struct ServoExecution {
    uint8_t servoID;
    ServoMovement movement;
    unsigned long stepStartTime;
    int currentStep;
    bool isActive;
  };
  
  ServoExecution servoExecs[group.servoCount];
  int activeServos = 0;
  
  // Inicializar cada servo del grupo
  for (int i = 0; i < group.servoCount; i++) {
    uint8_t servoID = group.servos[i];
    
    if (servoID < 32 && servoConfigs[servoID].enabled) {
      ServoMovement movement = servoConfigs[servoID].movement;
      
      if (movement.stepCount > 0) {
        servoExecs[activeServos] = {
          servoID,
          movement,
          startTime,
          0,
          true
        };
        activeServos++;
        
        Serial.print("  ➕ Servo ");
        Serial.print(servoID);
        Serial.print(" - ");
        Serial.print(servoConfigs[servoID].name);
        Serial.print(" | Pasos: ");
        Serial.print(movement.stepCount);
        Serial.println(" - EJECUTANDO EN PARALELO");
      }
    }
  }
  
  if (activeServos == 0) {
    Serial.println("❌ No hay servos válidos en el grupo");
    return;
  }
  
  Serial.print("🔄 INICIANDO PARALELISMO REAL: ");
  Serial.print(activeServos);
  Serial.println(" servos activos");
  
  // 🟢 BUCLE PRINCIPAL DE EJECUCIÓN PARALELA
  bool allServosFinished = false;
  
  while (!allServosFinished) {
    allServosFinished = true;
    unsigned long currentTime = millis();
    
    // Actualizar cada servo en paralelo
    for (int i = 0; i < activeServos; i++) {
      if (!servoExecs[i].isActive) continue;
      
      ServoExecution& exec = servoExecs[i];
      ServoMovement& movement = exec.movement;
      
      if (exec.currentStep < movement.stepCount) {
        allServosFinished = false;
        
        unsigned long stepElapsed = currentTime - exec.stepStartTime;
        uint16_t stepTime = movement.times[exec.currentStep];
        uint16_t stepDelay = movement.delays[exec.currentStep];
        
        // 🟢 FASE 1: Movimiento del paso actual
        if (stepElapsed < stepTime) {
          // Calcular progreso del movimiento
          float progress = (float)stepElapsed / (float)stepTime;
          progress = easeInOutCubic(progress);
          
          // Obtener ángulo actual
          int startAngle = (exec.currentStep == 0) ? 0 : movement.angles[exec.currentStep - 1];
          int targetAngle = movement.angles[exec.currentStep];
          int currentAngle = startAngle + (targetAngle - startAngle) * progress;
          
          // Mover servo a posición actual
          int pulse = angleToPulseINJORA(currentAngle);
          setServoPulse(exec.servoID, pulse);
        }
        // 🟢 FASE 2: Delay después del movimiento
        else if (stepElapsed < stepTime + stepDelay) {
          // Mantener posición final del paso
          int targetAngle = movement.angles[exec.currentStep];
          int pulse = angleToPulseINJORA(targetAngle);
          setServoPulse(exec.servoID, pulse);
        }
        // 🟢 FASE 3: Avanzar al siguiente paso
        else {
          exec.currentStep++;
          exec.stepStartTime = currentTime;
          
          if (exec.currentStep < movement.stepCount) {
            Serial.print("    Servo ");
            Serial.print(exec.servoID);
            Serial.print(" -> Paso ");
            Serial.print(exec.currentStep + 1);
            Serial.print(": ");
            Serial.print(movement.angles[exec.currentStep]);
            Serial.println("°");
          }
        }
      } else {
        // Servo completó todos sus pasos
        exec.isActive = false;
      }
    }
    
    // Pequeña pausa para no saturar
    delay(10);
  }
  
  Serial.println("✅ EJECUCIÓN PARALELA COMPLETADA - TODOS LOS SERVOS TERMINARON");
  
  // Apagar servos
  for (int i = 0; i < activeServos; i++) {
    setServoPulse(servoExecs[i].servoID, 0);
  }
}

// 🎯 HANDLER SIMPLIFICADO
void handleTestGroupExact() {
  if (server.hasArg("id")) {
    int groupId = server.arg("id").toInt();
    
    if (groupId >= 0 && groupId < programConfig.groupCount) {
      Group group = programConfig.groups[groupId];
      
      Serial.print("🎯 TEST EXACTO GRUPO: ");
      Serial.println(group.name);
      
      // Ejecutar reproducción exacta
      executeGroupExactSimultaneous(groupId);
      
      server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Reproducción exacta completada\"}");
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Grupo no encontrado\"}");
    }
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Falta parámetro id\"}");
  }
}


void handleSaveGroup() {
  if (server.hasArg("id") && server.hasArg("name") && server.hasArg("servos")) {
    int groupId = server.arg("id").toInt();
    String name = server.arg("name");
    String servosStr = server.arg("servos");
    
    Serial.print("Guardando grupo - ID: ");
    Serial.print(groupId);
    Serial.print(", Nombre: ");
    Serial.print(name);
    Serial.print(", Servos: ");
    Serial.println(servosStr);
    
    if (name.length() > 0) {
      int actualGroupId = groupId;
      
      if (groupId >= programConfig.groupCount) {
        actualGroupId = programConfig.groupCount;
        programConfig.groupCount++;
      } else {
        // Edición: No aumentar count, solo overwrite
      }
      
      if (actualGroupId >= 6 || programConfig.groupCount > 6) {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Máximo 6 grupos permitidos\"}");
        return;
      }
      
      strncpy(programConfig.groups[actualGroupId].name, name.c_str(), 14);
      programConfig.groups[actualGroupId].name[14] = '\0';
      programConfig.groups[actualGroupId].enabled = true;
      
      programConfig.groups[actualGroupId].servoCount = 0;
      int startPos = 0;
      for (int i = 0; i < servosStr.length() && programConfig.groups[actualGroupId].servoCount < 8; i++) {
        if (servosStr.charAt(i) == ',' || i == servosStr.length() - 1) {
          int endPos = (i == servosStr.length() - 1) ? i + 1 : i;
          String servoStr = servosStr.substring(startPos, endPos);
          int servoId = servoStr.toInt();
          if (servoId >= 0 && servoId < 32) {
            programConfig.groups[actualGroupId].servos[programConfig.groups[actualGroupId].servoCount] = servoId;
            programConfig.groups[actualGroupId].servoCount++;
          }
          startPos = i + 1;
        }
      }
      
      EEPROM.put(PROGRAM_EEPROM_OFFSET, programConfig);
      EEPROM.commit();
      
      Serial.print("Grupo guardado exitosamente - ID: ");
      Serial.print(actualGroupId);
      Serial.print(", Nombre: ");
      Serial.println(name);
      
      server.send(200, "application/json", "{\"status\":\"success\",\"actualId\":" + String(actualGroupId) + "}");
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Nombre inválido\"}");
    }
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Faltan parámetros\"}");
  }
}

// Eliminar grupo
void handleDeleteGroup() {
  if (server.hasArg("id")) {
    int groupId = server.arg("id").toInt();
    
    if (groupId >= 0 && groupId < programConfig.groupCount) {
      // Desplazar grupos
      for (int i = groupId; i < programConfig.groupCount - 1; i++) {
        programConfig.groups[i] = programConfig.groups[i + 1];
      }
      programConfig.groupCount--;
      
      // Guardar en EEPROM
      EEPROM.put(PROGRAM_EEPROM_OFFSET, programConfig);
      EEPROM.commit();
      
      Serial.print("Grupo eliminado: ");
      Serial.println(groupId);
      
      server.send(200, "application/json", "{\"status\":\"success\"}");
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"ID de grupo inválido\"}");
    }
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Falta parámetro id\"}");
  }
}

// === HANDLER PARA INICIAR LOOP ===
void handleStartLoop() {
  // Verificar que hay una secuencia cargada
  if (currentSequence.blockCount == 0) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No hay secuencia cargada\"}");
    return;
  }
  
  systemRunning = true;
  sequenceExecution = true;
  programStartTime = millis();
  lastLoopTime = millis();
  currentStep = 0;
  
  // Usar la secuencia actual para ejecución
  executionSequence = currentSequence;
  totalSteps = executionSequence.blockCount;
  
  Serial.println("🎬 INICIANDO EJECUCIÓN DE SECUENCIA");
  Serial.print("📋 Secuencia: ");
  Serial.println(executionSequence.sequenceName);
  Serial.print("📦 Bloques: ");
  Serial.println(totalSteps);
  Serial.print("🚨 Sensor PIR: ");
  Serial.println(execConfig.useSensor ? "ACTIVADO" : "DESACTIVADO");
  
  server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Ejecución iniciada\"}");
}

// === HANDLER PARA DETENER LOOP ===
void handleStopLoop() {
  systemRunning = false;
  sequenceExecution = false;
  pirTriggered = false;
  currentStep = 0;
  totalSteps = 0;
  
  // Mover todos los servos a posición central y apagar
  returnToCenterAndOff();
  
  Serial.println("⏹️ EJECUCIÓN DETENIDA");
  
  server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Ejecución detenida\"}");
}

void handleGetCurrentSequence() {
  String json = "{";
  json += "\"sequenceName\":\"";
  json += String(currentSequence.sequenceName);
  json += "\",\"blockCount\":";
  json += String(currentSequence.blockCount);
  json += ",\"totalDuration\":";
  json += String(currentSequence.totalDuration);
  json += "}";
  
  server.send(200, "application/json", json);
}


// Obtener configuración de ejecución
void handleGetExecConfig() {
  String json = "{";
  json += "\"loopInterval\":";
  json += String(execConfig.loopInterval);
  json += ",\"useSensor\":";
  json += execConfig.useSensor ? "true" : "false";
  json += ",\"currentMode\":";
  json += String(execConfig.currentMode);
  json += "}";
  
  server.send(200, "application/json", json);
}

void handleSaveExecConfig() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    
    // Parsear JSON simple
    if (body.indexOf("\"loopInterval\"") != -1) {
      int start = body.indexOf("\"loopInterval\":") + 15;
      int end = body.indexOf(",", start);
      if (end == -1) end = body.indexOf("}", start);
      execConfig.loopInterval = body.substring(start, end).toInt();
    }
    
    if (body.indexOf("\"useSensor\"") != -1) {
      execConfig.useSensor = (body.indexOf("\"useSensor\":true") != -1);
    }
    
    // Guardar en EEPROM
    EEPROM.put(EXEC_CONFIG_EEPROM_OFFSET, execConfig);
    EEPROM.commit();
    
    Serial.print("✅ Config guardada - Intervalo: ");
    Serial.print(execConfig.loopInterval);
    Serial.print("ms, Sensor: ");
    Serial.println(execConfig.useSensor ? "SI" : "NO");
    
    server.send(200, "application/json", "{\"status\":\"success\"}");
  } else {
    server.send(400, "application/json", "{\"status\":\"error\"}");
  }
} 

// Probar grupo - EJECUTA PASO A PASO SIMULTÁNEO


void debugEEPROM() {
  Serial.println("=== DEBUG EEPROM ===");
  Serial.print("Tamaño ServoConfig: ");
  Serial.println(sizeof(ServoConfig));
  Serial.print("Tamaño ProgramConfig: ");
  Serial.println(sizeof(ProgramConfig));
  Serial.print("Offset PROGRAM: ");
  Serial.println(PROGRAM_EEPROM_OFFSET);
  Serial.print("Group Count: ");
  Serial.println(programConfig.groupCount);
  
  for (int i = 0; i < programConfig.groupCount; i++) {
    Serial.print("Grupo ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(programConfig.groups[i].name);
    Serial.print(" (");
    Serial.print(programConfig.groups[i].servoCount);
    Serial.println(" servos)");
  }
  Serial.println("====================");
}


// Obtener lista de todas las secuencias guardadas
void handleGetSequences() {
  String json = "[";
  // Por ahora solo devolvemos la secuencia actual
  // Luego puedes expandir para múltiples secuencias
  json += "{\"id\":0,\"name\":\"" + String(currentSequence.sequenceName) + "\",\"blockCount\":" + String(currentSequence.blockCount) + "}";
  json += "]";
  server.send(200, "application/json", json);
}

// Cargar secuencia por ID
void handleLoadSequenceById() {
  if (server.hasArg("id")) {
    int sequenceId = server.arg("id").toInt();
    
    // Por ahora solo tenemos una secuencia
    if (sequenceId == 0) {
      String json = "{\"blocks\":[";
      for (int i = 0; i < currentSequence.blockCount; i++) {
        json += "{";
        json += "\"type\":\"";
        if (currentSequence.blocks[i].type == 0) json += "servo";
        else if (currentSequence.blocks[i].type == 1) json += "group";
        else if (currentSequence.blocks[i].type == 2) json += "behavior";
        json += "\",\"targetID\":";
        json += String(currentSequence.blocks[i].targetID);
        json += ",\"name\":\"";
        json += String(currentSequence.blocks[i].name);
        json += "\",\"duration\":";
        json += String(currentSequence.blocks[i].duration);
        json += ",\"behavior\":";
        if (currentSequence.blocks[i].type == 2) {
          json += "\"";
          if (currentSequence.blocks[i].behavior == 0) json += "sequential";
          else if (currentSequence.blocks[i].behavior == 1) json += "simultaneous";
          else if (currentSequence.blocks[i].behavior == 2) json += "random";
          json += "\"";
        } else {
          json += "null";
        }
        json += "}";
        if (i < currentSequence.blockCount - 1) json += ",";
      }
      json += "],\"sequenceName\":\"" + String(currentSequence.sequenceName) + "\",\"blockCount\":" + String(currentSequence.blockCount) + "}";
      
      server.send(200, "application/json", json);
    } else {
      server.send(404, "application/json", "{\"status\":\"error\",\"message\":\"Secuencia no encontrada\"}");
    }
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Falta ID\"}");
  }
}

// Eliminar secuencia
void handleDeleteSequence() {
  if (server.hasArg("id")) {
    int sequenceId = server.arg("id").toInt();
    
    // Por ahora solo tenemos una secuencia (ID 0)
    if (sequenceId == 0) {
      // Resetear la secuencia actual
      currentSequence.blockCount = 0;
      strncpy(currentSequence.sequenceName, "", 19);
      
      // Guardar en EEPROM
      EEPROM.put(SEQUENCE_EEPROM_OFFSET, currentSequence);
      EEPROM.commit();
      
      server.send(200, "application/json", "{\"status\":\"success\"}");
      Serial.println("🗑️ Secuencia eliminada de EEPROM");
    } else {
      server.send(404, "application/json", "{\"status\":\"error\",\"message\":\"Secuencia no encontrada\"}");
    }
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Falta ID\"}");
  }
}



void setup() {
  Serial.begin(115200);

  while (!Serial) {
    delay(100);
  }

  printSystemInfo();
  
  Serial.println("=== OPTICA BOAVISTA ===");
  
  // Inicializar EEPROM PRIMERO
  initEEPROM();

 

  // Inicializar variables de ejecución (antes del auto-start para no sobrescribir)
  systemRunning = false;
  sequenceExecution = false;
  pirTriggered = false;
  currentStep = 0;
  totalSteps = 0;

  // === NUEVO: Ejecutar automáticamente la secuencia guardada al encender ===
  if (currentSequence.blockCount > 0) {
    Serial.println("🔄 Recuperando y ejecutando secuencia guardada automáticamente al encender");
    startSequenceExecution();
  }

  // Inicializar sensor PIR
  pinMode(PIR_SENSOR_PIN, INPUT);
  Serial.println("✅ Sensor PIR inicializado en GPIO 7");

  // Cargar configuración de ejecución
  EEPROM.get(EXEC_CONFIG_EEPROM_OFFSET, execConfig);
  if (execConfig.loopInterval == 0 || execConfig.loopInterval > 60000) {
    execConfig.loopInterval = 5000;
    execConfig.useSensor = false;
    execConfig.currentMode = 0;
    Serial.println("✅ Configuración de ejecución inicializada con valores por defecto");
  }
  
  // CARGAR configuración de servos desde EEPROM
  loadServoConfig();
  
  // Inicializar I2C
  Wire.begin(8, 9);
  
  debugEEPROM();

  // Configurar PCA9685
  setupPCA9685();
  
  // Configurar WiFi
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(ssid, password);

  // Configurar rutas (las existentes más las mejoradas)
  server.on("/", handleRoot);
  server.on("/setup", handleSetup);
  server.on("/program", handleProgram);
  server.on("/detect", handleDetect);
  server.on("/getconfig", handleGetConfig);
  server.on("/savename", HTTP_POST, handleSaveName);
  server.on("/savesetup", HTTP_POST, handleSaveSetup);
  server.on("/servosweep", HTTP_POST, handleServoSweep);
  server.on("/getmovements", handleGetMovements);
  server.on("/savemovement", HTTP_POST, handleSaveMovement);
  server.on("/testmovement", HTTP_POST, handleTestMovement);
  server.on("/reseteeprom", HTTP_POST, handleResetEEPROM);
  server.on("/redetect", handleRedetect);
  server.on("/getgroups", handleGetGroups);
  server.on("/savegroup", HTTP_POST, handleSaveGroup);
  server.on("/deletegroup", HTTP_POST, handleDeleteGroup);
  server.on("/testgroup", HTTP_POST, handleTestGroup);
  server.on("/groups", handleGroups);
  server.on("/control", handleControl);
  server.on("/getcontrolstatus", handleGetControlStatus);
  server.on("/startloop", HTTP_POST, handleStartLoop);  // ← MEJORADO
  server.on("/stoploop", HTTP_POST, handleStopLoop);    // ← MEJORADO
  server.on("/getexecconfig", handleGetExecConfig);
  server.on("/saveexecconfig", HTTP_POST, handleSaveExecConfig);
  server.on("/sequenceeditor", handleSequenceEditor);
  server.on("/savesequence", HTTP_POST, handleSaveSequence);
  server.on("/loadsequence", handleLoadSequence);
  server.on("/testsequence", HTTP_POST, handleTestSequence);
  server.on("/getsequences", handleGetSequences);
  server.on("/loadsequence", handleLoadSequenceById);
  server.on("/deletesequence", HTTP_DELETE, handleDeleteSequence);
  server.on("/getcontrolstatus", handleGetControlStatus);
  server.on("/getcurrentsequence", handleGetCurrentSequence);
  server.on("/getservostatus", handleGetServoStatus);
  server.on("/testgroupexact", HTTP_POST, handleTestGroupExact);

  
  server.begin();
  Serial.println("Servidor: http://192.168.4.1");
}

// === LOOP PRINCIPAL MEJORADO ===
void loop() {
  server.handleClient();
  
  // Ejecución en loop si está activa
  if (systemRunning) {
    executeProgramLoop();
  }
}
