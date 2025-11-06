#include <Arduino.h>
#include <NimBLEDevice.h>

short powerInstantaneous = 0;
short cadenceInstantaneous = 0;
short speedInstantaneous = 0;
float powerScale = 1.28;
short resistance = 200; // Resistência inicial em %
bool notify = false;
bool trainingStarted = false;

// UUIDs para Client (conectar ao sensor)
static BLEUUID serviceUUID("1826"); // Fitness Machine
static BLEUUID charUUID("2ad2"); // Indoor Bike Data

static BLEUUID HRserviceUUID("180D"); // HR Service
static BLEUUID HRcharUUID("2a37"); // HR Measurement

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLEAdvertisedDevice *myDevice;

/* 
 * Server Stuff - Nosso dispositivo como servidor BLE
 */
static NimBLEServer *pServer;

// Declarar as características como variáveis globais
NimBLECharacteristic *CyclingPowerFeature = NULL;
NimBLECharacteristic *CyclingPowerMeasurement = NULL;
NimBLECharacteristic *CyclingPowerSensorLocation = NULL;
NimBLECharacteristic *FitnessMachineFeature = NULL;
NimBLECharacteristic *IndoorBikeData = NULL;
NimBLECharacteristic *FitnessMachineControlPoint = NULL;
NimBLECharacteristic *FitnessMachineStatus = NULL;

/** Callbacks do servidor */
class ServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer)
  {
    Serial.println("Client connected");
    Serial.println("Multi-connect support: start advertising");
    NimBLEDevice::startAdvertising();
  };

  void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc)
  {
    Serial.print("Client address: ");
    Serial.println(NimBLEAddress(desc->peer_ota_addr).toString().c_str());
    pServer->updateConnParams(desc->conn_handle, 24, 48, 0, 60);
  };

  void onDisconnect(NimBLEServer *pServer)
  {
    Serial.println("Client disconnected - start advertising");
    NimBLEDevice::startAdvertising();
    trainingStarted = false; // Reset training state on disconnect
  };

  void onMTUChange(uint16_t MTU, ble_gap_conn_desc *desc)
  {
    Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, desc->conn_handle);
  };
};

/** Callbacks para características gerais */
class CharacteristicCallbacks : public NimBLECharacteristicCallbacks
{
  void onRead(NimBLECharacteristic *pCharacteristic)
  {
    Serial.print(pCharacteristic->getUUID().toString().c_str());
    Serial.print(": onRead(), value: ");
    Serial.println(pCharacteristic->getValue().c_str());
  };

  void onWrite(NimBLECharacteristic *pCharacteristic)
  {
    Serial.print(pCharacteristic->getUUID().toString().c_str());
    Serial.print(": onWrite(), value: ");
    Serial.println(pCharacteristic->getValue().c_str());
  };

  void onNotify(NimBLECharacteristic *pCharacteristic)
  {
    Serial.println("Sending notification to clients");
  };

  void onStatus(NimBLECharacteristic *pCharacteristic, Status status, int code)
  {
    String str = ("Notification/Indication status code: ");
    str += status;
    str += ", return code: ";
    str += code;
    str += ", ";
    str += NimBLEUtils::returnCodeToString(code);
    Serial.println(str);
  };

  void onSubscribe(NimBLECharacteristic *pCharacteristic, ble_gap_conn_desc *desc, uint16_t subValue)
  {
    String str = "Client ID: ";
    str += desc->conn_handle;
    str += " Address: ";
    str += std::string(NimBLEAddress(desc->peer_ota_addr)).c_str();
    if (subValue == 0)
    {
      str += " Unsubscribed to ";
    }
    else if (subValue == 1)
    {
      str += " Subscribed to notifications for ";
    }
    else if (subValue == 2)
    {
      str += " Subscribed to indications for ";
    }
    else if (subValue == 3)
    {
      str += " Subscribed to notifications and indications for ";
    }
    str += std::string(pCharacteristic->getUUID()).c_str();
    Serial.println(str);
  };
};

void updateFitnessMachineStatus() {
    unsigned char status[3];
    status[0] = 0x0A; // Resistance Level Status type
    status[1] = resistance & 0xFF; // Current resistance
    status[2] = 0x00;
    
    if (FitnessMachineStatus) {
        FitnessMachineStatus->setValue(status, 3);
        FitnessMachineStatus->notify();
    }
}

/** Callbacks para o Control Point (ESSENCIAL para controle) */
class ControlPointCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        Serial.printf("Callback Control Point - Written value length: %d\n", value.length());
        
        if (value.length() > 0) {
            uint8_t opCode = value[0];
            Serial.printf("Control Point Command: 0x%02X\n", opCode);
            
            switch (opCode) {
                case 0x00: // Request Control
                    Serial.println("Request Control received");
                    sendControlResponse(0x00, 0x01); // opCode, result success
                    break;
                    
                case 0x04: // Set Target Resistance Level
                    if (value.length() >= 2) {
                        resistance = value[1];
                        Serial.printf("Set resistance to: %d%%\n", resistance);
                        sendControlResponse(0x04, 0x01); // success
                        updateFitnessMachineStatus();
                    }
                    break;
                    
                case 0x05: // Start training
                    if (value.length() >= 2) {
                        trainingStarted = (value[1] == 0x01);
                        Serial.printf("Training %s\n", trainingStarted ? "STARTED" : "STOPPED");
                        sendControlResponse(0x05, 0x01); // success
                    }
                    break;
                    
                case 0x07: // Set Indoor Bike Simulation Parameters
                    Serial.println("Simulation Parameters received (ignored)");
                    sendControlResponse(0x07, 0x01); // success
                    break;
                    
                default:
                    Serial.printf("Unhandled opcode: 0x%02X\n", opCode);
                    sendControlResponse(opCode, 0x80); // opcode not supported
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

/** Callbacks para descritores */
class DescriptorCallbacks : public NimBLEDescriptorCallbacks
{
  void onWrite(NimBLEDescriptor *pDescriptor)
  {
    Serial.print("Descriptor written value:");
    Serial.println(pDescriptor->getStringValue().c_str());
  };

  void onRead(NimBLEDescriptor *pDescriptor)
  {
    Serial.print(pDescriptor->getUUID().toString().c_str());
    Serial.println(" Descriptor read");
  };
};

/* 
 * Client Stuff - Conectar ao sensor real
 */
static void notifyCallback(
    BLERemoteCharacteristic *pBLERemoteCharacteristic,
    uint8_t *pData,
    size_t length,
    bool isNotify)
{
  powerInstantaneous = pData[8] | pData[9] << 8;
  cadenceInstantaneous = 60; // Simulado - substitua pelo valor real se disponível
  Serial.printf("Power = %d | Cadence = %d | Resistance = %d\n", powerInstantaneous, cadenceInstantaneous, resistance);
}

class MyClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient *pclient)
  {
  }

  void onDisconnect(BLEClient *pclient)
  {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer()
{
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient *pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  pClient->connect(myDevice);
  Serial.println(" - Connected to server");

  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr)
  {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr)
  {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  if (pRemoteCharacteristic->canRead())
  {
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());
  }

  if (pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->subscribe(true, notifyCallback);

  connected = true;
  return true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice *advertisedDevice)
  {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice->toString().c_str());

    if (advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(serviceUUID))
    {
      BLEDevice::getScan()->stop();
      myDevice = advertisedDevice;
      doConnect = true;
      doScan = true;
    }
  }
};

unsigned char bleBuffer[8];
unsigned short revolutions = 0;
unsigned short timestamp = 0;
long lastNotify = 0;
long lastRevolution = 0;

// Funções para Fitness Machine Service
void updateFitnessData() {
    unsigned char bikeData[10];
    
    // Flags (0x44 = Instantaneous Cadence + Instantaneous Power present)
    bikeData[0] = 0x44;
    bikeData[1] = 0x00;
    
    // Instantaneous Cadence (RPM) - 2 bytes
    bikeData[2] = cadenceInstantaneous & 0xFF;
    bikeData[3] = (cadenceInstantaneous >> 8) & 0xFF;
    
    // Instantaneous Power (Watts) - 2 bytes
    bikeData[4] = powerInstantaneous & 0xFF;
    bikeData[5] = (powerInstantaneous >> 8) & 0xFF;
    
    // Resistance Level (%) - 2 bytes
    bikeData[6] = resistance & 0xFF;
    bikeData[7] = 0x00;
    
    if (IndoorBikeData) {
        IndoorBikeData->setValue(bikeData, 8);
        IndoorBikeData->notify();
    }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting NimBLE Server - Controllable Fitness Machine");

  /** Inicializa dispositivo BLE */
  NimBLEDevice::init("QZESP-CONTROLLABLE");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  /** ====================================================
   * FITNESS MACHINE SERVICE (1826) - PARA CONTROLE
   * ==================================================== */
  NimBLEService *pFitnessMachineService = pServer->createService("1826");

  // Fitness Machine Feature (2ACC)
  FitnessMachineFeature = pFitnessMachineService->createCharacteristic(
      "2ACC",
      NIMBLE_PROPERTY::READ
  );

  // Indoor Bike Data (2AD2)
  IndoorBikeData = pFitnessMachineService->createCharacteristic(
      "2AD2", 
      NIMBLE_PROPERTY::NOTIFY
  );

  // Fitness Machine Control Point (2AD9) - ESSENCIAL PARA CONTROLE
  FitnessMachineControlPoint = pFitnessMachineService->createCharacteristic(
      "2AD9",
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE
  );

  // Fitness Machine Status (2ADA)
  FitnessMachineStatus = pFitnessMachineService->createCharacteristic(
      "2ADA",
      NIMBLE_PROPERTY::NOTIFY
  );

  // Configurar valores iniciais
  // Fitness Machine Features: 0x000000E0 = Indoor Bike + Power Measurement + Resistance Level
  unsigned char fitnessFeatures[4] = {0xE0, 0x00, 0x00, 0x00};
  FitnessMachineFeature->setValue(fitnessFeatures, 4);

  // Setar callbacks para o Control Point
  FitnessMachineControlPoint->setCallbacks(new ControlPointCallbacks());

  // Iniciar serviço
  pFitnessMachineService->start();

  /** ====================================================
   * CYCLING POWER SERVICE (1818) - PARA COMPATIBILIDADE
   * ==================================================== */
  NimBLEService *pCyclingPowerService = pServer->createService("1818");
  
  CyclingPowerFeature = pCyclingPowerService->createCharacteristic(
      "2A65",
      NIMBLE_PROPERTY::READ
  );
  
  CyclingPowerMeasurement = pCyclingPowerService->createCharacteristic(
      "2A63",
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  
  CyclingPowerSensorLocation = pCyclingPowerService->createCharacteristic(
      "2A5D",
      NIMBLE_PROPERTY::READ
  );

  // Cycling Power Features: 0x00000008 = Crank Revolution Data Supported
  unsigned char powerFeatures[4] = {0x08, 0x00, 0x00, 0x00};
  CyclingPowerFeature->setValue(powerFeatures, 4);

  // Sensor Location: 0x0D = Top of shoe
  unsigned char sensorLocation[1] = {0x0D};
  CyclingPowerSensorLocation->setValue(sensorLocation, 1);

  pCyclingPowerService->start();

  /** ====================================================
   * CONFIGURAR ADVERTISING
   * ==================================================== */
  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID("1826"); // Fitness Machine Service (PRINCIPAL)
  pAdvertising->addServiceUUID("1818"); // Cycling Power Service (compatibilidade)
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("Advertising Started - Device is now CONTROLLABLE");

  /** ====================================================
   * INICIALIZAR CLIENTE BLE (opcional - para conectar a sensor)
   * ==================================================== */
  Serial.println("Starting BLE Client...");
  BLEDevice::init("");

  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}

void loop()
{
  // Gerenciar conexão cliente (se necessário)
  if (doConnect == true)
  {
    if (connectToServer())
    {
      Serial.println("We are now connected to the BLE Server.");
    }
    else
    {
      Serial.println("We have failed to connect to the server; there is nothing more we will do.");
    }
    doConnect = false;
  }

  if (connected)
  {
    // Processar dados do sensor conectado (se aplicável)
  }
  else if (doScan)
  {
    BLEDevice::getScan()->start(0);
  }

  // Simular revoluções de pedalada baseado na cadência
  if (cadenceInstantaneous != 0 && (millis()) >= (lastRevolution + (60000 / cadenceInstantaneous)))
  {
    revolutions++;
    timestamp = (unsigned short)(((millis() * 1024) / 1000) % 65536);
    lastRevolution = millis();
  }

  // Atualizar dados a cada segundo
  if (millis() - lastNotify >= 1000)
  {
    // Atualizar dados do Fitness Machine Service
    updateFitnessData();
    
    // Manter compatibilidade com Cycling Power Service
    bleBuffer[0] = 0x20; // Flags: Crank Revolution Data present
    bleBuffer[1] = 0x00;
    bleBuffer[2] = powerInstantaneous & 0xFF;
    bleBuffer[3] = (powerInstantaneous >> 8) & 0xFF;
    bleBuffer[4] = revolutions & 0xFF;
    bleBuffer[5] = (revolutions >> 8) & 0xFF;
    bleBuffer[6] = timestamp & 0xFF;
    bleBuffer[7] = (timestamp >> 8) & 0xFF;
    
    if (CyclingPowerMeasurement) {
        CyclingPowerMeasurement->setValue(bleBuffer, 8);
        CyclingPowerMeasurement->notify();
    }
    
    lastNotify = millis();
    
    // Debug info
    Serial.printf("Power: %dW | Cadence: %dRPM | Resistance: %d%% | Training: %s\n", 
                  powerInstantaneous, cadenceInstantaneous, resistance, 
                  trainingStarted ? "ON" : "OFF");
  }
}