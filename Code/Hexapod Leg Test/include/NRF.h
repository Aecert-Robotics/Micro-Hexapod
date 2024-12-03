#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <vectors.h>

extern RF24 radio;

extern unsigned long rc_send_interval;

struct Data_Package {
    byte type; // 1 byte
    
    byte joy1_X; // 1 byte
    byte joy1_Y; // 1 byte
    
    byte joy2_X; // 1 byte
    byte joy2_Y; // 1 byte  
    byte slider1; // 1 byte
    byte slider2; // 1 byte

    byte joy1_Button:1; // 1 bit
    byte joy2_Button:1; // 1 bit
    byte pushButton1:1; // 1 bit
    byte pushButton2:1; // 1 bit
    byte idle:1;        // 1 bit
    byte sleep:1;        // 1 bit
    byte dynamic_stride_length:1; // 1 bit
    byte reserved : 1;  // 1 bits padding, 1 byte total

    byte gait;  // 1 byte
};

extern Data_Package dataPackage;

void setupNRF();
void recieveNRFData();
void RC_DisplayData();