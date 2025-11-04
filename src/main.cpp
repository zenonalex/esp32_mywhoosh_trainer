#include <Arduino.h>   
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Services
#define FTMS_SERVICE_UUID        "1826"
#define CSC_SERVICE_UUID         "1816"
#define DIS_SERVICE_UUID         "180A"

BLEServer* pServer;
BLEService* pFtmsService;
BLEService* pCscService;
BLEService* pDisService;

bool deviceConnected = false;
bool controlGranted = false;
float currentResistance = 0.0;

// FTMS Features - Inclui Resistance Level
uint8_t ftmsFeatures[4] = {0xE0, 0x00, 0x00, 0x00}; // 0x000000E0

// ⚡⚡⚡ FUNÇÃO PARA CONTROLAR A RESISTÊNCIA DA BIKE ⚡⚡⚡ (DECLARADA ANTES)
void controlarResistenciaBike(float resistenciaPercent) {
  Serial.printf("🚴 IMPLEMENT: Control bike resistance to %.1f%%\n", resistenciaPercent);
  
  // AQUI VOCÊ ADICIONA O CÓDIGO PARA CONTROLAR SUA BIKE!
  // Exemplos:
  
  // 1. Para controle PWM (motor DC, servo, etc.):
  // int pwmValue = map(resistenciaPercent, 0, 100, 0, 255);
  // analogWrite(PINO_PWM, pwmValue);
  
  // 2. Para controle digital (relé, etc.):
  if (resistenciaPercent == 0) {
    Serial.println("   -> RESISTANCE: FREE");
    // digitalWrite(PINO_RELE, LOW);
  } else if (resistenciaPercent < 25) {
    Serial.println("   -> RESISTANCE: VERY LOW");
  } else if (resistenciaPercent < 50) {
    Serial.println("   -> RESISTANCE: LOW");
  } else if (resistenciaPercent < 75) {
    Serial.println("   -> RESISTANCE: MEDIUM");
  } else {
    Serial.println("   -> RESISTANCE: HIGH");
  }
  
  // 3. Para freio magnético:
  // controlarFreioMagnetico(resistenciaPercent);
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      controlGranted = false;
      Serial.println("🎯 TRAINERDAY CONNECTED!");
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      controlGranted = false;
      Serial.println("📵 Disconnected");
      delay(500);
      BLEDevice::startAdvertising();
      Serial.println("🔄 Advertising restarted");
    }
};

class MyControlCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() == 0) return;

      uint8_t opCode = value[0];
      uint8_t response[3];
      
      Serial.printf("📨 Command: 0x%02X - ", opCode);

      switch(opCode) {
        case 0x00: // Request Control
          Serial.println("Request Control");
          controlGranted = true;
          response[0] = 0x00;
          response[1] = 0x01; // Success
          response[2] = value[1];
          pCharacteristic->setValue(response, 3);
          pCharacteristic->notify();
          Serial.println("✅ Control GRANTED to TrainerDay");
          break;
          
        case 0x05: // ⚡⚡⚡ SET TARGET RESISTANCE LEVEL ⚡⚡⚡
          if (value.length() >= 3) {
            int16_t resistanceLevel = (value[2] << 8) | value[1]; // Little endian
            currentResistance = resistanceLevel * 0.1f; // Convert to percentage
            
            Serial.printf("🎯 SET RESISTANCE: %d -> %.1f%%\n", resistanceLevel, currentResistance);
            
            // AQUI VOCÊ IMPLEMENTA O CONTROLE DA SUA BIKE!
            controlarResistenciaBike(currentResistance);
            
            response[0] = 0x05;
            response[1] = 0x01; // Success
            pCharacteristic->setValue(response, 2);
            pCharacteristic->notify();
          }
          break;
          
        case 0x06: // Set Target Power
          if (value.length() >= 3) {
            int16_t targetPower = (value[2] << 8) | value[1];
            Serial.printf("⚡ SET TARGET POWER: %d watts\n", targetPower);
            
            response[0] = 0x06;
            response[1] = 0x01;
            pCharacteristic->setValue(response, 2);
            pCharacteristic->notify();
          }
          break;
          
        case 0x11: // Set Indoor Bike Simulation Parameters
          if (value.length() >= 7) {
            int16_t windSpeed = (value[2] << 8) | value[1];
            int16_t grade = (value[4] << 8) | value[3];
            int16_t crr = (value[6] << 8) | value[5];
            Serial.printf("🏔️ Simulation: Wind=%d, Grade=%d, CRR=%d\n", windSpeed, grade, crr);
            
            response[0] = 0x11;
            response[1] = 0x01;
            pCharacteristic->setValue(response, 2);
            pCharacteristic->notify();
          }
          break;
          
        default:
          Serial.printf("Unknown: 0x%02X\n", opCode);
          break;
      }
    }
};

void sendBikeData() {
  if (!deviceConnected || !controlGranted) return;

  uint8_t bikeData[11];
  
  // Flags: More Data + Instantaneous Cadence + Instantaneous Power
  bikeData[0] = 0x24; 
  bikeData[1] = 0x00;
  
  // Instantaneous Power (Watts)
  uint16_t power = 50 + (millis() % 50); // Simula 50-100W
  bikeData[2] = power & 0xFF;
  bikeData[3] = (power >> 8) & 0xFF;
  
  // Instantaneous Cadence (RPM * 2)
  uint16_t cadence = 60 + (millis() % 40); // Simula 60-100 RPM
  uint16_t cadenceValue = cadence * 2;
  bikeData[4] = cadenceValue & 0xFF;
  bikeData[5] = (cadenceValue >> 8) & 0xFF;
  
  BLECharacteristic* pBikeChar = pFtmsService->getCharacteristic(BLEUUID("2ACE"));
  if (pBikeChar) {
    pBikeChar->setValue(bikeData, 6);
    pBikeChar->notify();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println();
  Serial.println("==========================================");
  Serial.println("   TRAINERDAY - RESISTANCE CONTROL");
  Serial.println("==========================================");
  Serial.println("Now with RESISTANCE control from TrainerDay!");
  Serial.println("==========================================");

  BLEDevice::init("SmartBike-Trainer");
  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // === DEVICE INFORMATION ===
  pDisService = pServer->createService(BLEUUID(DIS_SERVICE_UUID));
  
  BLECharacteristic* pManuf = pDisService->createCharacteristic(
    BLEUUID("2A29"), BLECharacteristic::PROPERTY_READ);
  pManuf->setValue("SmartBike");
  
  BLECharacteristic* pModel = pDisService->createCharacteristic(
    BLEUUID("2A24"), BLECharacteristic::PROPERTY_READ);
  pModel->setValue("FTMS-Controller");
  
  pDisService->start();

  // === FTMS SERVICE ===
  pFtmsService = pServer->createService(BLEUUID(FTMS_SERVICE_UUID));

  BLECharacteristic* pFeatureChar = pFtmsService->createCharacteristic(
    BLEUUID("2ACC"), BLECharacteristic::PROPERTY_READ);
  pFeatureChar->setValue(ftmsFeatures, 4);

  BLECharacteristic* pBikeDataChar = pFtmsService->createCharacteristic(
    BLEUUID("2ACE"), BLECharacteristic::PROPERTY_NOTIFY);
  pBikeDataChar->addDescriptor(new BLE2902());

  // Control Point - AGORA COM CALLBACK PARA RESISTÊNCIA
  BLECharacteristic* pControlChar = pFtmsService->createCharacteristic(
    BLEUUID("2AD9"), BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pControlChar->addDescriptor(new BLE2902());
  pControlChar->setCallbacks(new MyControlCallbacks()); // ⚡ IMPORTANTE!

  pFtmsService->start();

  // === CSC SERVICE ===
  pCscService = pServer->createService(BLEUUID(CSC_SERVICE_UUID));
  
  BLECharacteristic* pCscChar = pCscService->createCharacteristic(
    BLEUUID("2A5B"), BLECharacteristic::PROPERTY_NOTIFY);
  pCscChar->addDescriptor(new BLE2902());
  
  pCscService->start();

  // === ADVERTISING ===
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  
  BLEAdvertisementData advData;
  advData.setFlags(0x06);
  advData.setName("SmartBike-Trainer");
  advData.setCompleteServices(BLEUUID(FTMS_SERVICE_UUID));
  
  BLEAdvertisementData scanResp;
  scanResp.setAppearance(0x0340);
  scanResp.setName("SmartBike-Trainer");
  
  pAdvertising->setAdvertisementData(advData);
  pAdvertising->setScanResponseData(scanResp);
  pAdvertising->setScanResponse(true);
  
  BLEDevice::startAdvertising();

  Serial.println("✅ Ready for TrainerDay resistance control!");
  Serial.println("📡 Device: SmartBike-Trainer");
  Serial.println("🎯 TrainerDay should now send resistance commands");
  Serial.println("⚡ Resistance values will be shown in Serial Monitor");
  Serial.println("==========================================");

  pinMode(2, OUTPUT);
}

void loop() {
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink > 1000) {
    digitalWrite(2, !digitalRead(2));
    
    if (!deviceConnected) {
      static uint32_t counter = 0;
      if (counter % 5 == 0) {
        Serial.println("📡 Advertising for TrainerDay...");
      }
      counter++;
    } else {
      digitalWrite(2, HIGH);
      
      // Enviar dados de bike periodicamente
      static uint32_t lastData = 0;
      if (millis() - lastData > 1000 && controlGranted) {
        sendBikeData();
        Serial.printf("📊 Sending data - Resistance: %.1f%%\n", currentResistance);
        lastData = millis();
      }
    }
    
    lastBlink = millis();
  }
  
  delay(100);
}