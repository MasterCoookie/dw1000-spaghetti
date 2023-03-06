#include <SPI.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "DW1000Ranging.h"

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

#define SERVICE_UUID "50218d18-bc42-11ed-afa1-0242ac120002"
#define READ_CHARACTERISTIC_UUID "57eb6e60-bc42-11ed-afa1-0242ac120002"
#define WRITE_CHARACTERISTIC_UUID "5b28fd72-bc42-11ed-afa1-0242ac120002"

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin
int cycleCount = 9999;
int limiter = 200;
std::string anchorAddress;
char anchorAddressChar[5];
int numberOfRangingProtocols;
int serialInputLength = 0;
int inputLength = 0;

bool deviceConnected = false;
BLEServer* pServer;
BLEAdvertising* pAdvertising;
BLECharacteristic *pWriteCharacteristic;
BLECharacteristic *pReadCharacteristic;
bool receivedData = false;
String currentData;

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    Serial.println("New BLE device connected");
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) override {
    Serial.println("BLE device disconnected");
    deviceConnected = false;
    pAdvertising->start();
  }
};

class RemoteCallback: public BLECharacteristicCallbacks {
  public:
   void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string value = pCharacteristic->getValue();
    /*
    Serial.println();
    Serial.println("*********");
    Serial.print("New value: ");
*/
    String readData;
    for (int i = 0; i < value.length(); i++) {
      // Serial.print(value[i]);
      readData += value[i];
    }
    currentData = readData;
    receivedData = true;
    //Serial.println(readData);
  }
};

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("Initializing DW1000");
    //init the configuration
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
    //define the sketch as anchor. It will be great to dynamically change the type of module
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    //Enable the filter to smooth the distance
    //DW1000Ranging.useRangeFilter(true);

    //we start the module as a tag
  
    DW1000Ranging.initializeVariables(250, 10, true, 100);
    DW1000Ranging.startAsTag("FF:FB:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);
    //to make it run first time
    DW1000Ranging.setSentAck(true);
    DW1000Ranging.beginProtocol();
  
    Serial.println("DW1000 setup complete");
    
    //starting BLE
    Serial.println("Initializing BLE");
    BLEDevice::init("SPGH-TAG");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pWriteCharacteristic = pService->createCharacteristic(WRITE_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);
    pReadCharacteristic = pService->createCharacteristic(READ_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ);
    pWriteCharacteristic->setCallbacks(new RemoteCallback());
    pReadCharacteristic->setValue("Hello Dupa");

    pService->start();

    pAdvertising = pServer->getAdvertising();
    pAdvertising->start();

    Serial.println("BLE setup complete");
}

void initCom(String dataString) {
    inputLength = dataString.length();
    dataString.trim();
    char buf[inputLength+1];
    dataString.toCharArray(buf, inputLength+1);
    if(DW1000Ranging.decodeSerial(buf, inputLength)) {
      anchorAddress.clear();
      anchorAddress = DW1000Ranging.getAnchorAddressFromSerial();
      numberOfRangingProtocols = DW1000Ranging.getRangingProtocolNumber();
      strcpy(anchorAddressChar, anchorAddress.c_str());
      limiter = numberOfRangingProtocols;
      cycleCount = 0;
      DW1000Ranging.setCycleCounter();
    }
}

void loop()
{
  if(cycleCount < limiter) {   
    DW1000Ranging.loop_tag(anchorAddressChar, pReadCharacteristic);
    cycleCount = DW1000Ranging.getCycleCounter();
    //Serial.println(cycleCount);
    // DW1000Ranging.loop();
  }
  else if(receivedData)
  {
    receivedData = false;
    initCom(currentData);
  }
  /*else if(Serial.available() != 0) {
    String serialString = Serial.readString();
    serialInputLength = serialString.length();
    serialString.trim();
    char buf[serialString.length()+1];
    serialString.toCharArray(buf, serialString.length()+1);
    if(DW1000Ranging.decodeSerial(buf, serialInputLength)) {
      //Serial.println("Success");
      anchorAddress.clear();
      anchorAddress = DW1000Ranging.getAnchorAddressFromSerial();
      numberOfRangingProtocols = DW1000Ranging.getRangingProtocolNumber();
      strcpy(anchorAddressChar, anchorAddress.c_str());
      //Serial.println(anchorAddressChar);
      //Serial.println(numberOfRangingProtocols);  
      limiter = numberOfRangingProtocols;
      cycleCount = 0;
      DW1000Ranging.setCycleCounter();
    }
  }*/
}

void newRange()
{
    Serial.print("from: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
    Serial.print("\t Range: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRange());
    Serial.print(" m");
    Serial.print("\t RX power: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
    Serial.println(" dBm");
}

void newDevice(DW1000Device *device)
{
    Serial.print("ranging init; 1 device added ! -> ");
    Serial.print(" short:");
    Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
    Serial.print("delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);
}