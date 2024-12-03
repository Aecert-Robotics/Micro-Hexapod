#include <Arduino.h>
#include <NRF.h>
#include <SPI.h>
#include <RF24.h>

RF24 radio(46, 47); // CE, CSN

uint8_t address[6] = "HEX01";
Data_Package dataPackage;

void setupNRF()
{

    pinMode(10, OUTPUT);

    radio.begin();
    radio.setAddressWidth(5);
    radio.setPALevel(RF24_PA_LOW);
    radio.setPayloadSize(32); // Set the payload size to the maximum of 32 bytes
    radio.setChannel(124);
    radio.openReadingPipe(1, address);
    radio.startListening();
    Serial.println("Radio Started");

    // initialize dataPackage values
    dataPackage.joy1_Y = 0;
    Serial.println("Data Initialized");
}

void recieveNRFData()
{
    if (radio.available())
    {
        radio.read(&dataPackage, sizeof(dataPackage));
        RC_DisplayData();
    }
}

void RC_DisplayData(){
  Serial.print("Joy1 X: ");
  Serial.print(dataPackage.joy1_X);

  Serial.print(" | Joy1 Y: ");
  Serial.print(dataPackage.joy1_Y);

  Serial.print(" | Joy1 Button: ");
  Serial.print(dataPackage.joy1_Button);

  Serial.print(" | Joy2 X: ");
  Serial.print(dataPackage.joy2_X);

  Serial.print(" | Joy2 Y: ");
  Serial.print(dataPackage.joy2_Y);

  Serial.print(" | Joy2 Button: ");
  Serial.print(dataPackage.joy2_Button);

  Serial.print(" | Pot 1: ");
  Serial.print(dataPackage.slider1);

  Serial.print(" | Pot 2: ");
  Serial.print(dataPackage.slider2);

  Serial.print(" | Button 1: ");
  Serial.print(dataPackage.pushButton1);

  Serial.print(" | Button 2: ");
  Serial.println(dataPackage.pushButton2);
}