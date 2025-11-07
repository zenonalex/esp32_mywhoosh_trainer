#include <Arduino.h>
#include <NimBLEDevice.h>

// ============================================================================
// CONFIGURAÇÕES DOS SENSORES REAIS
// ============================================================================

const std::string CADENCE_SENSOR_NAME = "53470-17";
const std::string SPEED_SENSOR_NAME = "29562-49";
static NimBLEUUID CSC_SERVICE_UUID("00001816-0000-1000-8000-00805f9b34fb");
static NimBLEUUID CSC_CHARACTERISTIC_UUID("00002a5b-0000-1000-8000-00805f9b34fb");
const float WHEEL_CIRCUMFERENCE = 2.096f;

NimBLEAdvertisedDevice *cadenceDevice = nullptr;
NimBLEAdvertisedDevice *speedDevice = nullptr;
NimBLEClient *cadenceClient = nullptr;
NimBLEClient *speedClient = nullptr;

uint16_t lastCadenceRevs = 0;
uint16_t lastCadenceTime = 0;
unsigned long lastCadenceMillis = 0;

uint32_t lastWheelRevs = 0;
uint16_t lastWheelTime = 0;
unsigned long lastSpeedMillis = 0;

// ============================================================================
// VARIÁVEIS PARA SERVIDOR BLE (MYWHOOSH) - TODAS COMO short
// ============================================================================

short powerInstantaneous = 200; // Potência inicial de 200W
short cadenceInstantaneous = 80;
short speedInstantaneous = 2000; // 20.00 km/h
short resistance = 8;           // Resistência inicial 8%
bool trainingStarted = false;

static NimBLEServer *pServer;
NimBLECharacteristic *IndoorBikeData = NULL;
NimBLECharacteristic *FitnessMachineControlPoint = NULL;
NimBLECharacteristic *FitnessMachineStatus = NULL;
NimBLECharacteristic *CyclingPowerMeasurement = NULL;

// ============================================================================
// PROCESSAMENTO DOS SENSORES
// ============================================================================

void processCadenceData(uint8_t *data, size_t length)
{
  if (length < 5)
    return;

  uint8_t flags = data[0];
  if (flags & 0x02)
  {
    uint16_t revs = data[1] | (data[2] << 8);
    uint16_t time = data[3] | (data[4] << 8);

    if (lastCadenceTime != 0 && time != lastCadenceTime)
    {
      uint16_t deltaRevs = revs - lastCadenceRevs;
      uint16_t deltaTime = time - lastCadenceTime;

      if (deltaTime > 0)
      {
        float seconds = deltaTime / 1024.0f;
        float rpm = (deltaRevs / seconds) * 60.0f;

        if (rpm >= 0 && rpm <= 200)
        {
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
  if (length < 7)
    return;

  uint8_t flags = data[0];
  if (flags & 0x01)
  {
    uint32_t revs = (uint32_t)data[1] | ((uint32_t)data[2] << 8) | ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 24);
    uint16_t time = data[5] | (data[6] << 8);

    if (lastWheelTime != 0)
    {
      uint32_t deltaRevs = revs - lastWheelRevs;
      uint16_t deltaTime = time - lastWheelTime;

      if (deltaTime > 0x8000)
      {
        deltaTime = 65535 - lastWheelTime + time;
      }

      if (deltaTime > 0)
      {
        float seconds = deltaTime / 1024.0f;
        float distanceMeters = deltaRevs * WHEEL_CIRCUMFERENCE;
        float speedKmh = (distanceMeters / seconds) * 3.6f;

        if (speedKmh >= 0 && speedKmh <= 100)
        {
          speedInstantaneous = (short)(speedKmh * 100); // Em centésimos de km/h

          // Cálculo de potência baseado na velocidade e resistência
          powerInstantaneous = (short)(speedKmh * resistance * 0.8f);
          if (powerInstantaneous < 0)
            powerInstantaneous = 0;
          if (powerInstantaneous > 1000)
            powerInstantaneous = 1000;

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

  if (client == cadenceClient)
  {
    processCadenceData(data, length);
  }
  else if (client == speedClient)
  {
    processSpeedData(data, length);
  }
}

class SensorScanCallbacks : public NimBLEAdvertisedDeviceCallbacks
{
  void onResult(NimBLEAdvertisedDevice *advertisedDevice) override
  {
    std::string name = advertisedDevice->getName();
    if (!cadenceDevice && name.find(CADENCE_SENSOR_NAME) != std::string::npos)
    {
      Serial.printf("🚴‍♂️ Sensor de cadência encontrado: %s\n", advertisedDevice->getAddress().toString().c_str());
      cadenceDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
    }
    else if (!speedDevice && name.find(SPEED_SENSOR_NAME) != std::string::npos)
    {
      Serial.printf("⚙️ Sensor de velocidade encontrado: %s\n", advertisedDevice->getAddress().toString().c_str());
      speedDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
    }

    if (cadenceDevice && speedDevice)
    {
      NimBLEDevice::getScan()->stop();
      Serial.println("✅ Ambos sensores encontrados!");
    }
  }
};

bool connectToSensor(NimBLEAdvertisedDevice *device, const char *label)
{
  Serial.printf("🔗 Conectando ao sensor %s...\n", label);

  NimBLEClient *client = NimBLEDevice::createClient();
  client->setConnectTimeout(10);

  if (!client->connect(device))
  {
    Serial.printf("❌ Falha ao conectar ao sensor %s\n", label);
    NimBLEDevice::deleteClient(client);
    return false;
  }

  Serial.printf("✅ Conectado ao sensor %s!\n", label);

  NimBLERemoteService *service = client->getService(CSC_SERVICE_UUID);
  if (!service)
  {
    Serial.printf("❌ Serviço não encontrado no sensor %s\n", label);
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  NimBLERemoteCharacteristic *characteristic = service->getCharacteristic(CSC_CHARACTERISTIC_UUID);
  if (!characteristic)
  {
    Serial.printf("❌ Característica não encontrada no sensor %s\n", label);
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  if (characteristic->canNotify())
  {
    characteristic->subscribe(true, handleSensorNotification);
    Serial.printf("📡 Notificações ativadas para %s\n", label);
  }

  if (strcmp(label, "cadência") == 0)
  {
    cadenceClient = client;
  }
  else
  {
    speedClient = client;
  }

  return true;
}

void startSensorScan()
{
  Serial.println("🔍 Procurando sensores...");
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->clearResults();
  scan->setAdvertisedDeviceCallbacks(new SensorScanCallbacks(), false);
  scan->setActiveScan(true);
  scan->start(5, false);
}

// ============================================================================
// SERVIDOR BLE - COM short E FORMATO QUE VOCÊ PEDIU
// ============================================================================

class ServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer)
  {
    Serial.println("📱 MyWhoosh conectado!");
    NimBLEDevice::startAdvertising();
  };

  void onDisconnect(NimBLEServer *pServer)
  {
    Serial.println("📱 MyWhoosh desconectado");
    NimBLEDevice::startAdvertising();
    trainingStarted = false;
  };
};

class ControlPointCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0)
    {
      uint8_t opCode = value[0];
      Serial.printf("🎮 Comando recebido: 0x%02X\n", opCode);

      switch (opCode)
      {
      case 0x00: // Request Control
        sendControlResponse(0x00, 0x01);
        break;

      case 0x04: // Set Target Resistance Level
        if (value.length() >= 2)
        {
          resistance = value[1];
          Serial.printf("🎯 Resistência definida: %d%%\n", resistance);
          sendControlResponse(0x04, 0x01);

          // Atualizar status de resistência
          uint8_t status[3] = {0x0A, (uint8_t)resistance, 0x00};
          FitnessMachineStatus->setValue(status, 3);
          FitnessMachineStatus->notify();
        }
        break;

      case 0x05: // Start/Stop training
        if (value.length() >= 2)
        {
          trainingStarted = (value[1] == 0x01);
          Serial.printf("🏁 Treino %s\n", trainingStarted ? "INICIADO" : "PARADO");
          sendControlResponse(0x05, 0x01);
        }
        break;

      case 0x07: // Simulation Parameters
        sendControlResponse(0x07, 0x01);
        break;

      default:
        sendControlResponse(opCode, 0x80);
        break;
      }
    }
  }

private:
  void sendControlResponse(uint8_t opCode, uint8_t result)
  {
    uint8_t response[3] = {0x80, opCode, result};
    FitnessMachineControlPoint->setValue(response, 3);
    FitnessMachineControlPoint->indicate();
  }
};

void updateIndoorBikeData()
{
  unsigned char data[16];
  data[0] = 0x44;
  data[1] = 0x00;

  // ⚡ TESTE: DATA[2-3] como RESISTÊNCIA
  data[2] = resistance & 0xFF;           // Resistência low
  data[3] = (resistance >> 8) & 0xFF;    // Resistência high
  
  // DATA[4-5] como CADÊNCIA
  short cadenceValue = cadenceInstantaneous * 2;
  data[4] = cadenceValue & 0xFF;         // Cadência low
  data[5] = (cadenceValue >> 8) & 0xFF;  // Cadência high

  // ⚡ DATA[6-7] como POTÊNCIA
  data[6] = powerInstantaneous & 0xFF;    // Potência low
  data[7] = (powerInstantaneous >> 8) & 0xFF; // Potência high

  IndoorBikeData->setValue(data, 8);
  IndoorBikeData->notify();

  Serial.printf("🔍 TESTE INVERTIDO - Resistência: %d%% | Cadência: %dRPM | Potência: %dW\n",
                resistance, cadenceInstantaneous, powerInstantaneous);
}

void updateCyclingPowerData()
{
  unsigned char data[8]; // ⚡ MANTER unsigned char

  // Flags (0x10 = Instantaneous Power present)
  data[0] = 0x10;
  data[1] = 0x00;

  // Instantaneous Power (Watts) - LITTLE ENDIAN
  short powerValue = powerInstantaneous;
  data[2] = powerValue & 0xFF;
  data[3] = (powerValue >> 8) & 0xFF;

  CyclingPowerMeasurement->setValue(data, 4);
  CyclingPowerMeasurement->notify();

  Serial.printf("⚡ CyclingPower - Potência: %dW\n", powerInstantaneous);
}

void setupServer()
{
  Serial.println("🚀 Iniciando servidor BLE...");
  NimBLEDevice::init("QZ-POWER-TRAINER");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // ===== FITNESS MACHINE SERVICE =====
  NimBLEService *pFitnessService = pServer->createService("1826");

  // Fitness Machine Feature
  NimBLECharacteristic *pFitnessFeature = pFitnessService->createCharacteristic("2ACC", NIMBLE_PROPERTY::READ);
  uint8_t fitnessFeatures[4] = {0xE0, 0x00, 0x00, 0x00}; // Indoor Bike + Power + Resistance
  pFitnessFeature->setValue(fitnessFeatures, 4);

  // Indoor Bike Data - PRINCIPAL PARA POTÊNCIA
  IndoorBikeData = pFitnessService->createCharacteristic("2AD2", NIMBLE_PROPERTY::NOTIFY);

  // Fitness Machine Control Point
  FitnessMachineControlPoint = pFitnessService->createCharacteristic("2AD9", NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
  FitnessMachineControlPoint->setCallbacks(new ControlPointCallbacks());

  // Fitness Machine Status
  FitnessMachineStatus = pFitnessService->createCharacteristic("2ADA", NIMBLE_PROPERTY::NOTIFY);

  pFitnessService->start();

  // ===== CYCLING POWER SERVICE =====
  NimBLEService *pPowerService = pServer->createService("1818");

  // Cycling Power Feature
  NimBLECharacteristic *pPowerFeature = pPowerService->createCharacteristic("2A65", NIMBLE_PROPERTY::READ);
  uint8_t powerFeatures[4] = {0x08, 0x00, 0x00, 0x00};
  pPowerFeature->setValue(powerFeatures, 4);

  // Cycling Power Measurement - ALTERNATIVO PARA POTÊNCIA
  CyclingPowerMeasurement = pPowerService->createCharacteristic("2A63", NIMBLE_PROPERTY::NOTIFY);

  pPowerService->start();

  // ===== ADVERTISING =====
  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID("1826");
  pAdvertising->addServiceUUID("1818");
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("✅ Servidor BLE pronto! Potência: 200W inicial");
}

// ============================================================================
// SETUP E LOOP
// ============================================================================

void setup()
{
  Serial.begin(115200);
  Serial.println("\n🚴‍♂️ ESP32 MyWhoosh Trainer - Com short e formato original");

  setupServer();
  delay(2000);
  startSensorScan();
}

void loop()
{
  static unsigned long lastSensorManage = 0;
  static unsigned long lastDataUpdate = 0;

  // Gerenciar sensores
  if (millis() - lastSensorManage > 2000)
  {
    if (cadenceDevice && !cadenceClient)
    {
      connectToSensor(cadenceDevice, "cadência");
    }
    if (speedDevice && !speedClient)
    {
      connectToSensor(speedDevice, "velocidade");
    }

    if ((!cadenceDevice || !speedDevice) && !NimBLEDevice::getScan()->isScanning())
    {
      startSensorScan();
    }
    lastSensorManage = millis();
  }

  // Atualizar dados BLE - ENVIAR POTÊNCIA AQUI
  if (millis() - lastDataUpdate >= 250)
  {
    updateIndoorBikeData();   // Envia potência via Fitness Machine
    updateCyclingPowerData(); // Envia potência via Cycling Power

    lastDataUpdate = millis();
  }

  // Verificar timeouts
  if (cadenceClient && (millis() - lastCadenceMillis > 10000))
  {
    cadenceClient->disconnect();
    cadenceClient = nullptr;
  }

  if (speedClient && (millis() - lastSpeedMillis > 10000))
  {
    speedClient->disconnect();
    speedClient = nullptr;
  }

  delay(100);
}