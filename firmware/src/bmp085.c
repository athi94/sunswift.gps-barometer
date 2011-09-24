/* ------------------------------------------------------------------------
   Barometer BMP085
   Functions for the I2C communications of the barometer
   Calculations based off the datsheet: "BMP085 Digitial Pressure Sensor Datsheet"
   refer to page 13

   File name: bmp085.c
   Author: Robert Makepeace
   Date: 22/9/2011
   ------------------------------------------------------------------------- */
#include "bmp085.h"

#include <sys/types.h>

#include <project/rtc_mcp79410.h>
#include <project/driver_config.h>
#include <project/target_config.h>

#include <arch/i2c.h>
#include <scandal/engine.h>
#include <scandal/message.h>
#include <scandal/leds.h>
#include <scandal/utils.h>
#include <scandal/uart.h>
#include <scandal/stdio.h>


//can check EOC pin if communication has finished instead of abartary delays.
//Busy: 0, Ready:1
int main (void) {
  long b5, pres, temp, alt, up, ut;
  Bmp085_cal barometerCal = readCalibrationValues();		//read calibration values
  while(1) {
    ut = bmp085ReadUT();					//read uncompensated temperature
    scandal_naive_delay(4500);					//delay 4.5ms
    up = bmp085ReadUP();					//read uncompensated pressure
    scandal_naive_delay(4500);					//delay 4.5ms    
    b5 = bmp085(ut, barometerCal);				//calculate temperature constant
    temp = bmp085GetTemperature(ut, b5);			//calculate true temperature
    pres = bmp085GetPressure(up, barometerCal, b5);		//calculate true pressure
    alt = bmp085GetAltitude(pres);				//estimate the altitude
  }
  return 0;
}

//Read the internal calibration values of the barometer and returns structure of constants
//All 16 bit values: msb is at first address, lsb at second address
Bmp085_cal readCalibrationValues(void) {
    Bmp085_cal barometerCal;
    barometerCal -> AC1 = bmp085ReadShort(AC1_REG); //0xAA - msb, 0xAB - lsb
    barometerCal -> AC2 = bmp085ReadShort(AC2_REG); //0xAC - msb, 0xAD - lsb
    barometerCal -> AC3 = bmp085ReadShort(AC3_REG); //0xAE - msb, 0xAF - lsb
    barometerCal -> AC4 = bmp085ReadShort(AC4_REG); //0xB0 - msb, 0xB1 - lsb
    barometerCal -> AC5 = bmp085ReadShort(AC5_REG); //0xB2 - msb, 0xB3 - lsb
    barometerCal -> AC6 = bmp085ReadShort(AC6_REG); //0xB4 - msb, 0xB5 - lsb
    barometerCal -> B1 = bmp085ReadShort(B1_REG);   //0xB6 - msb, 0xB7 - lsb
    barometerCal -> B2 = bmp085ReadShort(B2_REG);   //0xB8 - msb, 0xB9 - lsb
    barometerCal -> MB = bmp085ReadShort(MB_REG);   //0xBA - msb, 0xBB - lsb
    barometerCal -> MC = bmp085ReadShort(MC_REG);   //0xBC - msb, 0xBC - lsb
    barometerCal -> MD = bmp085ReadShort(MD_REG);   //0xBE - msb, 0xBd - lsb
    return barometerCal;
}
// Read the uncompensated temperature value (16 bit)
// need to wait 4.5 ms after function
long bmp085ReadUT(void) {
    long ut;
    bmp085WriteByte(CTL_REG, TEMP_READ);	//writes 0x23 into 0xF4
    ut = (long)bmp085ReadShort(DATA_REG);	//0xF6 - msb, 0xF7 - lsb
    return ut;
}

// Read the uncompensated pressure value (19 bit)
// need to wait 4.5 ms after function
//Depends on the OSS - over sampling setting define in the header file
long bmp085ReadUP(void) {
    long ut;
    bmp085WriteByte(CTL_REG, PRES_READ);	//writes (0x34 + (OSS << 6)) into 0xF4 
    ut = bmp085Read24bit(DATA_REG);			//0xF6 - msb, 0xF7 - lsb, 0xF8 - xlsb
    ut = ut >> (8 - OSS);
    return ut;
}

//Calculate the temperature constant b5 (needed for temp and pressure calculations.
long bmp085Getb5((long ut,Bmp085_cal barometerCal) {
  long x1, x2, b5;
  x1 = (((long)ut - (long)barometerCal -> AC6)*(long)barometerCal -> AC5) >> 15;
  x2 = ((long)barometerCal -> MC << 11)/(x1 + barometerCal -> MD);
  b5 = x1 + x2;
  return b5;
}

// Calculate true temperature given uncompensated temperature.
// Value returned will be in units of 0.1 deg C
// b5 is also required so bmp085Getb5() must be called first.
long bmp085GetTemperature(long ut,long b5) {
  long temp;
  temp = ((b5 + 8)>>4);
  return temp;
}
   
// Calculate true pressure given uncompensated pressure
// Value returned will be pressure in units of Pa.
// b5 is also required so bmp085Getb5() must be called first.
long bmp085GetPressure(long up, Bmp085_cal barometerCal, long b5) {
  long x1, x2, x3, b3, b6, pres;
  unsigned long b4, b7;
    
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (barometerCal -> AC2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)barometerCal -> AC1)*4 + x3)<<OSS) + 2)>>2;
  
  // Calculate B4
  x1 = (barometerCal -> AC3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (barometerCal -> AC4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    pres = (b7<<1)/b4;
  else
    pres = (b7/b4)<<1;
    
  x1 = (pres>>8) * (pres>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * pres)>>16;
  pres += (x1 + x2 + 3791)>>4;
  
  return pres;
}

//Approximate the altitude from the pressure
//needs a power function!!!
long bmp085GetAltitude(long pres) {
  long altitude
  long tmp;
  
  tmp = pres/PRESSURE_SEALEVEL;
  tmp = 1 - pow(tmp, 5.255);//need to add a power function.
  altitude = 44300 * temp;
  return altitude;
}

//Read a short - 16 bit number
//address refers to the first msb, lsb is the next one.
long bmp085Read24bit(unsigned char address){ 
    int i; 
    long buffer;
    I2CWriteLength = 3;
    I2CReadLength = 3;
    I2CMasterBuffer[0] = BMP_MODULE_ADDRESS |BMP_WRITE; //0xEE
    I2CMasterBuffer[1] = address; // Start read register
    I2CMasterBuffer[2] = BMP_MODULE_ADDRESS |BMP_READ; //0xEF
    // Put if statement here!
    I2CEngine();
    buffer = (I2CSlaveBuffer[0] << 16) | (I2CSlaveBuffer[1] << 8) | (I2CSlaveBuffer[2])
    if(buffer != 0x000000 && buffer != 0xFFFFFF) {
      //ERROR!!!!!
    }
    return buffer;
}

//Read a short - 16 bit number
//address refers to the first msb, lsb is the next one.
short bmp085ReadShort(unsigned char address){ 
    int i; 
    short buffer;
    I2CWriteLength = 3;
    I2CReadLength = 2;
    I2CMasterBuffer[0] = BMP_MODULE_ADDRESS |BMP_WRITE; //0xEE
    I2CMasterBuffer[1] = address; // Start read register
    I2CMasterBuffer[2] = BMP_MODULE_ADDRESS |BMP_READ; //0xEF
    // Put if statement here!
    I2CEngine();
    buffer = (I2CSlaveBuffer[0] << 8) | (I2CSlaveBuffer[1])
    if(buffer != 0x0000 && buffer != 0xFFFF) {
      //ERROR!!!!!
    }
    return buffer;
}
  
//Read a byte - 8bits  
//Module addres: 0xEE
//Read: 0xEF
//Write: 0xEE
unsigned char bmp085ReadByte(unsigned char address) { 
    int i; 
    unsigned char byte
    I2CWriteLength = 3;
    I2CReadLength = 1;
    I2CMasterBuffer[0] = BMP_MODULE_ADDRESS |BMP_WRITE; //0xEE
    I2CMasterBuffer[1] = address; // Start read register
    I2CMasterBuffer[2] = BMP_MODULE_ADDRESS |BMP_READ; //0xEF
    // Put if statement here!
    I2CEngine();
    byte = I2CSlaveBuffer[0];
    if(byte != 0x00 && byte != 0xFF) {
      //ERROR!!!!!
      byte = 0x00;
    }
    return byte;
}

//Write a byte - 8bits
//Module addres: 0xEE
//Read: 0xEF
//Write: 0xEE
void bmp085WriteByte(unsigned char address, unsigned char value) { 
    I2CWriteLength = 3;
    I2CReadLength = 0;
    I2CMasterBuffer[0] = BMP_MODULE_ADDRESS |BMP_WRITE; //0xEE
    I2CMasterBuffer[1] = address; 
    I2CMasterBuffer[2] = value;
    // Put if statement here!
    I2CEngine();
}