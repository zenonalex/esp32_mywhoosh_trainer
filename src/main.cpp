#include <Arduino.h>
#include <NimBLEDevice.h>

// ============================================================================
// CONFIGURAÇÕES DOS SENSORES REAIS
// ============================================================================

// 🔧 Identificadores dos sensores BLE
const std::string CADENCE_SENSOR_NAME = "53470-17";
const std::string SPEED_SENSOR_NAME = "29562-49";

// ⚙️ UUIDs do serviço e característica CSC (Cycling Speed and Cadence)
static NimBLEUUID CSC_SERVICE_UUID("00001816-0000-1000-8000-00805f9b34fb");
static NimBLEUUID CSC_CHARACTERISTIC_UUID("00002a5b-0000-1000-8000-00805f9b34fb");

// ⚙️ Configuração da roda (em metros)
const float WHEEL_CIRCUMFERENCE = 2.096f;

// 🔢 Referências de dispositivos e clientes
NimBLEAdvertisedDevice *cadenceDevice = nullptr;
NimBLEAdvertisedDevice *speedDevice = nullptr;
NimBLEClient *cadenceClient = nullptr;
NimBLEClient *speedClient = nullptr;

// 📊 Variáveis para cálculo de cadência
uint16_t lastCadenceRevs = 0;
uint16_t lastCadenceTime = 0;
unsigned long lastCadenceMillis = 0;

// 📊 Variáveis para cálculo de velocidade
uint32_t lastWheelRevs = 0;
uint16_t lastWheelTime = 0;
unsigned long lastSpeedMillis = 0;

// ============================================================================
// VARIÁVEIS PARA SERVIDOR BLE (MYWHOOSH)
// ============================================================================

short powerInstantaneous = 0;
short cadenceInstantaneous = 0;
short speedInstantaneous = 0;
short resistance = 200;
bool trainingStarted = false;

// Server variables
static NimBLEServer *pServer;

// Características do servidor BLE
NimBLECharacteristic *CyclingPowerFeature = NULL;
NimBLECharacteristic *CyclingPowerMeasurement = NULL;
NimBLECharacteristic *CyclingPowerSensorLocation = NULL;
NimBLECharacteristic *FitnessMachineFeature = NULL;
NimBLECharacteristic *IndoorBikeData = NULL;
NimBLECharacteristic *FitnessMachineControlPoint = NULL;
NimBLECharacteristic *FitnessMachineStatus = NULL;

// ============================================================================
// PROCESSAMENTO DOS SENSORES REAIS
// ============================================================================

void processCadenceData(uint8_t *data, size_t length)
{
  if (length < 5) return;

  uint8_t flags = data[0];
  if (flags & 0x02) { // contém dados de pedivela
    uint16_t revs = data[1] | (data[2] << 8);
    uint16_t time = data[3] | (data[4] << 8);

    if (lastCadenceTime != 0 && time != lastCadenceTime) {
      uint16_t deltaRevs = revs - lastCadenceRevs;
      uint16_t deltaTime = time - lastCadenceTime;
      
      if (deltaTime > 0) {
        float seconds = deltaTime / 1024.0f;
        float rpm = (deltaRevs / seconds) * 60.0f;
        
        if (rpm >= 0 && rpm <= 200) {
          cadenceInstantaneous = (short)rpm;
          Serial.printf("🚴‍♂️ Cadência: %.1f RPM\n", rpm);
        }
      }
    }

    lastCadenceRevs = revs;
    lastCadenceTime = time;
    lastCadenceMillis = millis();
  }
}

void processSpeedData(uint8_t *data, size_t length)
{
  if (length < 7) return;

  uint8_t flags = data[0];
  if (flags & 0x01) { // contém dados de roda
    uint32_t revs = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
    uint16_t time = data[5] | (data[6] << 8);

    if (lastWheelTime != 0 && time != lastWheelTime) {
      uint32_t deltaRevs = revs - lastWheelRevs;
      uint16_t deltaTime = time - lastWheelTime;
      
      if (deltaTime > 0) {
        float seconds = deltaTime / 1024.0f;
        float distanceMeters = deltaRevs * WHEEL_CIRCUMFERENCE;
        float speedKmh = (distanceMeters / seconds) * 3.6f;
        
        if (speedKmh >= 0 && speedKmh <= 100) {
          speedInstantaneous = (short)(speedKmh * 10);
          powerInstantaneous = (short)(speedKmh * resistance * 0.1f);
          Serial.printf("⚙️ Velocidade: %.2f km/h | Potência: %d W\n", speedKmh, powerInstantaneous);
        }
      }
    }

    lastWheelRevs = revs;
    lastWheelTime = time;
    lastSpeedMillis = millis();
  }
}

// ============================================================================
// CALLBACKS E CONEXÃO DOS SENSORES
// ============================================================================

void handleSensorNotification(NimBLERemoteCharacteristic *characteristic, uint8_t *data, size_t length, bool isNotify)
{
  NimBLEClient *client = characteristic->getRemoteService()->getClient();

  if (client == cadenceClient) {
    processCadenceData(data, length);
  } else if (client == speedClient) {
    processSpeedData(data, length);
  }
}

class SensorScanCallbacks : public NimBLEAdvertisedDeviceCallbacks
{
  void onResult(NimBLEAdvertisedDevice *advertisedDevice) override
  {
    std::string name = advertisedDevice->getName();
    if (!cadenceDevice && name.find(CADENCE_SENSOR_NAME) != std::string::npos) {
      Serial.printf("🚴‍♂️ Sensor de cadência encontrado: %s\n", advertisedDevice->getAddress().toString().c_str());
      cadenceDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
    } else if (!speedDevice && name.find(SPEED_SENSOR_NAME) != std::string::npos) {
      Serial.printf("⚙️ Sensor de velocidade encontrado: %s\n", advertisedDevice->getAddress().toString().c_str());
      speedDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
    }

    if (cadenceDevice && speedDevice) {
      NimBLEDevice::getScan()->stop();
      Serial.println("✅ Ambos sensores encontrados. Parando scan.");
    }
  }
};

bool connectToSensor(NimBLEAdvertisedDevice *device, const char *label)
{
  Serial.printf("🔗 Tentando conectar ao sensor %s (%s)...\n", label, device->getAddress().toString().c_str());

  NimBLEClient *client = NimBLEDevice::createClient();
  
  // Configurações otimizadas para conexão
  client->setConnectTimeout(15); // Aumentar timeout
  client->setConnectionParams(12, 12, 0, 600); // Parâmetros mais conservadores
  
  if (!client->connect(device, false)) { // auto=false para controle manual
    Serial.printf("⚠️ Falha ao conectar ao sensor %s.\n", label);
    NimBLEDevice::deleteClient(client);
    return false;
  }

  Serial.printf("✅ Conectado ao sensor %s!\n", label);

  // Pequena pausa após conexão
  delay(100);

  NimBLERemoteService *service = client->getService(CSC_SERVICE_UUID);
  if (!service) {
    Serial.printf("❌ Serviço CSC não encontrado no sensor %s.\n", label);
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  // Pequena pausa antes de buscar característica
  delay(50);

  NimBLERemoteCharacteristic *characteristic = service->getCharacteristic(CSC_CHARACTERISTIC_UUID);
  if (!characteristic) {
    Serial.printf("❌ Característica CSC não encontrada no sensor %s.\n", label);
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  if (characteristic->canNotify()) {
    if (!characteristic->subscribe(true, handleSensorNotification, true)) {
      Serial.printf("❌ Falha ao subscrever notificações do sensor %s.\n", label);
      client->disconnect();
      NimBLEDevice::deleteClient(client);
      return false;
    }
    Serial.printf("📡 Subscrito para notificações CSC (%s)\n", label);
  }

  if (std::string(label) == "cadência") {
    cadenceClient = client;
    lastCadenceRevs = 0;
    lastCadenceTime = 0;
  } else {
    speedClient = client;
    lastWheelRevs = 0;
    lastWheelTime = 0;
  }

  return true;
}

void startSensorScan()
{
  Serial.println("🔍 Iniciando scan BLE para sensores...");
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->clearResults(); // Limpar resultados anteriores
  scan->setAdvertisedDeviceCallbacks(new SensorScanCallbacks(), false);
  scan->setActiveScan(true);
  scan->setInterval(1349);
  scan->setWindow(449);
  scan->setMaxResults(0);
  scan->start(15, false); // Scan mais longo
}

// ============================================================================
// SERVIDOR BLE (MYWHOOSH)
// ============================================================================

class ServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer) {
    Serial.println("Client connected to MyWhoosh server");
    NimBLEDevice::startAdvertising();
  };

  void onDisconnect(NimBLEServer *pServer) {
    Serial.println("MyWhoosh client disconnected - start advertising");
    NimBLEDevice::startAdvertising();
    trainingStarted = false;
  };
};

class ControlPointCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            uint8_t opCode = value[0];
            switch (opCode) {
                case 0x00: // Request Control
                    sendControlResponse(0x00, 0x01);
                    break;
                case 0x04: // Set Target Resistance Level
                    if (value.length() >= 2) {
                        resistance = value[1];
                        Serial.printf("Set resistance to: %d%%\n", resistance);
                        sendControlResponse(0x04, 0x01);
                    }
                    break;
                case 0x05: // Start training
                    if (value.length() >= 2) {
                        trainingStarted = (value[1] == 0x01);
                        Serial.printf("Training %s\n", trainingStarted ? "STARTED" : "STOPPED");
                        sendControlResponse(0x05, 0x01);
                    }
                    break;
                case 0x07: // Set Indoor Bike Simulation Parameters
                    sendControlResponse(0x07, 0x01);
                    break;
                default:
                    sendControlResponse(opCode, 0x80);
                    break;
            }
        }
    }
    
private:
    void sendControlResponse(uint8_t opCode, uint8_t result) {
        unsigned char response[3] = {0x80, opCode, result};
        if (FitnessMachineControlPoint) {
            FitnessMachineControlPoint->setValue(response, 3);
            FitnessMachineControlPoint->indicate();
        }
    }
};

void updateFitnessData() {
    unsigned char bikeData[16];
    bikeData[0] = 0x64; // Flags
    bikeData[1] = 0x00;
    
    uint16_t speedValue = speedInstantaneous;
    bikeData[2] = speedValue & 0xFF;
    bikeData[3] = (speedValue >> 8) & 0xFF;
    
    uint16_t cadenceValue = cadenceInstantaneous * 2;
    bikeData[4] = cadenceValue & 0xFF;
    bikeData[5] = (cadenceValue >> 8) & 0xFF;
    
    bikeData[6] = powerInstantaneous & 0xFF;
    bikeData[7] = (powerInstantaneous >> 8) & 0xFF;
    
    if (IndoorBikeData) {
        IndoorBikeData->setValue(bikeData, 8);
        IndoorBikeData->notify();
    }
}

void updateCyclingPowerData() {
    unsigned char powerData[8];
    powerData[0] = 0x20; // Flags
    powerData[1] = 0x00;
    powerData[2] = powerInstantaneous & 0xFF;
    powerData[3] = (powerInstantaneous >> 8) & 0xFF;
    
    static uint16_t cumulativeRevs = 0;
    cumulativeRevs += cadenceInstantaneous / 60;
    powerData[4] = cumulativeRevs & 0xFF;
    powerData[5] = (cumulativeRevs >> 8) & 0xFF;
    
    static uint16_t lastCrankTime = 0;
    lastCrankTime = (uint16_t)((millis() * 1024 / 1000) % 65536);
    powerData[6] = lastCrankTime & 0xFF;
    powerData[7] = (lastCrankTime >> 8) & 0xFF;
    
    if (CyclingPowerMeasurement) {
        CyclingPowerMeasurement->setValue(powerData, 8);
        CyclingPowerMeasurement->notify();
    }
}

void setupServer() {
  Serial.println("Starting NimBLE Server - Controllable Fitness Machine");
  NimBLEDevice::init("QZESP-CONTROLLABLE");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Fitness Machine Service
  NimBLEService *pFitnessMachineService = pServer->createService("1826");
  FitnessMachineFeature = pFitnessMachineService->createCharacteristic("2ACC", NIMBLE_PROPERTY::READ);
  IndoorBikeData = pFitnessMachineService->createCharacteristic("2AD2", NIMBLE_PROPERTY::NOTIFY);
  FitnessMachineControlPoint = pFitnessMachineService->createCharacteristic("2AD9", NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
  FitnessMachineStatus = pFitnessMachineService->createCharacteristic("2ADA", NIMBLE_PROPERTY::NOTIFY);

  unsigned char fitnessFeatures[4] = {0xE0, 0x00, 0x00, 0x00};
  FitnessMachineFeature->setValue(fitnessFeatures, 4);
  FitnessMachineControlPoint->setCallbacks(new ControlPointCallbacks());
  pFitnessMachineService->start();

  // Cycling Power Service
  NimBLEService *pCyclingPowerService = pServer->createService("1818");
  CyclingPowerFeature = pCyclingPowerService->createCharacteristic("2A65", NIMBLE_PROPERTY::READ);
  CyclingPowerMeasurement = pCyclingPowerService->createCharacteristic("2A63", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  CyclingPowerSensorLocation = pCyclingPowerService->createCharacteristic("2A5D", NIMBLE_PROPERTY::READ);

  unsigned char powerFeatures[4] = {0x08, 0x00, 0x00, 0x00};
  CyclingPowerFeature->setValue(powerFeatures, 4);
  unsigned char sensorLocation[1] = {0x0D};
  CyclingPowerSensorLocation->setValue(sensorLocation, 1);
  pCyclingPowerService->start();

  // Advertising
  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID("1826");
  pAdvertising->addServiceUUID("1818");
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("✅ Servidor BLE iniciado - Pronto para MyWhoosh");
}

// ============================================================================
// SETUP E LOOP PRINCIPAIS
// ============================================================================

void setup()
{
  Serial.begin(115200);
  Serial.println("🚴‍♂️ Iniciando ESP32 Dual BLE: Sensores + MyWhoosh");
  
  setupServer();
  
  // Pequena pausa antes de iniciar o scan
  delay(1000);
  startSensorScan();
}

void loop()
{
  static unsigned long lastSpeedConnectAttempt = 0;
  static int speedConnectAttempts = 0;

  // Gerenciar conexões com sensores
  if (cadenceDevice && !cadenceClient) {
    connectToSensor(cadenceDevice, "cadência");
  }
  
  if (speedDevice && !speedClient) {
    // Tentar conectar ao sensor de velocidade com intervalo crescente
    if (millis() - lastSpeedConnectAttempt > (2000 + (speedConnectAttempts * 1000))) {
      if (connectToSensor(speedDevice, "velocidade")) {
        speedConnectAttempts = 0;
      } else {
        speedConnectAttempts++;
        if (speedConnectAttempts > 5) {
          Serial.println("🔄 Muitas falhas - reiniciando scan...");
          speedConnectAttempts = 0;
          speedDevice = nullptr;
          startSensorScan();
        }
      }
      lastSpeedConnectAttempt = millis();
    }
  }

  // Scan periódico se sensores faltando
  static unsigned long lastScanTime = 0;
  if ((!cadenceDevice || !speedDevice) && !NimBLEDevice::getScan()->isScanning() && 
      (millis() - lastScanTime > 30000)) {
    Serial.println("🔁 Repetindo scan para sensores faltantes...");
    startSensorScan();
    lastScanTime = millis();
  }

  // Atualizar dados BLE
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 1000) {
    updateFitnessData();
    updateCyclingPowerData();
    
    Serial.printf("📊 STATUS: Cadência=%dRPM | Velocidade=%.1fkm/h | Potência=%dW | Resistência=%d%%\n", 
                  cadenceInstantaneous, speedInstantaneous / 10.0f, powerInstantaneous, resistance);
    
    lastUpdate = millis();
  }

  // Verificar timeouts
  if (cadenceClient && (millis() - lastCadenceMillis > 15000)) {
    Serial.println("⚠️ Sensor de cadência timeout - reconectando");
    cadenceClient->disconnect();
    NimBLEDevice::deleteClient(cadenceClient);
    cadenceClient = nullptr;
  }
  
  if (speedClient && (millis() - lastSpeedMillis > 15000)) {
    Serial.println("⚠️ Sensor de velocidade timeout - reconectando");
    speedClient->disconnect();
    NimBLEDevice::deleteClient(speedClient);
    speedClient = nullptr;
  }

  delay(100);
}