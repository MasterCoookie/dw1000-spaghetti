#include <SPI.h>
#include "DW1000Ranging.h"

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

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

void setup()
{
    Serial.begin(115200);
    delay(1000);
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
    DW1000Ranging.startAsTag("AB:02:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);
    //to make it run first time
    DW1000Ranging.setSentAck(true);
    DW1000Ranging.beginProtocol();
    
}

void loop()
{
  if(cycleCount < limiter) {   
    DW1000Ranging.loop_tag(anchorAddressChar);
    cycleCount = DW1000Ranging.getCycleCounter();
    //Serial.println(cycleCount);
    // DW1000Ranging.loop();
  }
  else if(Serial.available() != 0) {
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
  }
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