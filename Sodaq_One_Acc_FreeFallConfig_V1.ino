/* Example for using the accelerometer with the Sodaq_LSM303AGR library */
/* The example is tested on an Arduino M0 and a Sodaq Explorer with a Sodaq NB-IoT shield */
/* This code prints accelerometer readings every second and registers whether the board is flipped by interrupt */
#include <Arduino.h>
#include <Sodaq_LSM303AGR.h>
#include <Sodaq_RN2483.h>

#define ARDUINO_SODAQ_ONE
#define loraSerial Serial1
#define debugSerial SERIAL_PORT_MONITOR

// OTAA
uint8_t DevEUI[8] = { 0x19, 0x89, 0x19, 0x89, 0x19, 0x89, 0x19, 0x89 };
uint8_t AppEUI[8] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x01, 0x71, 0x83 };
uint8_t AppKey[16] = { 0x98, 0x42, 0xC1, 0x9F, 0x65, 0xB9, 0x8B, 0xDB, 0x54, 0x7E, 0xB9, 0xF8, 0x98, 0x65, 0x6A, 0x52 };


Sodaq_LSM303AGR AccMeter;

const uint8_t deviceAddress = SODAQ_LSM303AGR_ACCEL_ADDRESS;
const uint8_t accelSignalPin = ACCEL_INT1;

void writeAccelRegister(uint8_t reg, uint8_t value)
{
    SerialUSB.print("Writing to reg: ");
    SerialUSB.print(reg,HEX);
    SerialUSB.print(" the value of: ");
    SerialUSB.print(value, HEX);
    SerialUSB.print(" (");
    SerialUSB.print(value, BIN); 
    SerialUSB.println(")");
    Wire.beginTransmission(deviceAddress);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
    Wire.requestFrom(deviceAddress, 1);
}


uint8_t readAccelRegister(uint8_t reg)
{
    Wire.beginTransmission(deviceAddress);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom(deviceAddress, 1);
    return Wire.read();
}

long lastmillis;
void setup() {
  SerialUSB.begin(9600);
   loraSerial.begin(LoRaBee.getDefaultBaudRate());
    setupLoRaOTAA();
  while ((!SerialUSB) && (millis() < 10000)) {
    // Wait 10 seconds for the Serial Monitor
  }

  // Start the I2C bus

  Wire.begin();
  SerialUSB.print("talking to device: ");
  SerialUSB.println(deviceAddress,BIN);
  //AccMeter.rebootAccelerometer();
  delay(1000);
  SerialUSB.print("WHO_AM_I: ");
  SerialUSB.println(readAccelRegister(0x0F), BIN);

  writeAccelRegister(AccMeter.CTRL_REG1_A, 0xa7);      // XYZ on 100Hz
  writeAccelRegister(AccMeter.CTRL_REG2_A, 0x00);      // High pass filter off
  writeAccelRegister(AccMeter.CTRL_REG3_A, 0x40);      // Interrupt flagged on INT1
  writeAccelRegister(AccMeter.CTRL_REG4_A, 0x00);      // FS = 2g
  writeAccelRegister(AccMeter.CTRL_REG5_A, 0x08);      // Latch the interrupt
  writeAccelRegister(AccMeter.INT1_THS_A, /*0x16*/0b00000111);     // threshold = 350 mg
  writeAccelRegister(AccMeter.INT1_DURATION_A, 0x06);  // minimum event duration
  writeAccelRegister(AccMeter.INT1_CFG_A, 0x95);       // Free fall recognition on
/*
  pinMode(accelSignalPin, INPUT);                      // Make signal pin an input
  readAccelRegister(AccMeter.INT1_SRC_A);              // Reset interrupt latch
*/
  lastmillis = millis();
}

void setupLoRaOTAA() {
  if (LoRaBee.initOTA(loraSerial, DevEUI, AppEUI, AppKey, false))
  {
    debugSerial.println("Communication to LoRaBEE successful.");
  }
  else
  {
    debugSerial.println("OTAA Setup failed!");
  }
}


void setupLoRa() {

  // OTAA
  setupLoRaOTAA();
  LoRaBee.setSpreadingFactor(9);
}
uint16_t detectCount = 0;
bool event = false;
bool fall = false;
void interrupt_event(){
   if(millis() - lastmillis > 1000) {
   event = true;
   }
}

void interrupt_handling()
{
    // Do not print in an interrupt event when sleep is enabled.
    SerialUSB.println("event");

    readAccelRegister(AccMeter.INT1_SRC_A);
   if(millis() - lastmillis > 1000) {
    
        detectCount++;
        fall = true;
        SerialUSB.print(detectCount);
        SerialUSB.println("check: ");
        
        lastmillis = millis();
   }     
      
}


void loop() {
  #define ACC_INT_PIN 4
  pinMode(ACC_INT_PIN, INPUT);
  attachInterrupt(ACC_INT_PIN, interrupt_event, RISING);
  if(event) {

    interrupt_handling();
    if(fall == true){
      
      
      byte bytesToSend[2];
  
      bytesToSend[0] = detectCount >> 8;       // High byte Accelerometer
      bytesToSend[1] = detectCount ;           // Low Byte Accelerometer
      LoRaBee.sendReqAck(1, bytesToSend, 5, 3);
      fall = false;
    }
     event = false;

  }
 
     

}
