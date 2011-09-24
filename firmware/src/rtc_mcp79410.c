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

extern volatile uint32_t I2CCount;
extern volatile uint8_t I2CMasterBuffer[BUFSIZE];
extern volatile uint8_t I2CSlaveBuffer[BUFSIZE];
extern volatile uint32_t I2CMasterState;
extern volatile uint32_t I2CReadLength, I2CWriteLength;

#define ADDR_RTC_SRAM   0xDE
#define ADDR_EEPROM 	0xAE
#define ENABLE_OSC 	0x80
#define ENABLE_BAT 	0x08

void InitRTC(void){

    if ( I2CInit( (uint32_t)I2CMASTER ) == FALSE )  /* initialize I2c */
    {
     ;//I2C Failed to Initialise, To take care of this error later.
    }
    StartOsc();
    //scandal_delay(50);

    /* The following code sets the time on the RTC as we'd like
    char DateArray[6] = {2, 4, 0, 9, 1, 1}; //24/09/11 (Day:Mon:Yr)
    SetDate(&DateArray);
    char TimeArray[6] = {2, 2, 1, 0, 0, 1}; // 22:10:01 (Hr:Min:Sec)
    SetTime(&TimeArray);
    */    
}

int ReadTime(void){

    int sec, sec10; //Second in minute
    int min, min10; //Minute of hour
    int hr, hr10;   //Hour of day
    int dom, dom10;     //Date of month
    int mon, mon10;     //Month of year
    int year, year10;   //Year in XX format
    uint32_t rtc_dateArr[6], rtc_timeArr[6];

    I2CWriteLength = 2;
    I2CReadLength = 7;
    I2CMasterBuffer[0] = ADDR_RTC_SRAM;
    I2CMasterBuffer[1] = 0x00; // Start read register
    I2CMasterBuffer[2] = ADDR_RTC_SRAM | RD_BIT;
    // Put if statement here!
    I2CEngine();

    sec=    ((I2CSlaveBuffer[0]>>0) & 0x0F);
    sec10=  ((I2CSlaveBuffer[0]>>4) & 0x07);
    min=    ((I2CSlaveBuffer[1]>>0) & 0x0F);
    min10=  ((I2CSlaveBuffer[1]>>4) & 0x07);
    hr= ((I2CSlaveBuffer[2]>>0) & 0x0F);
    hr10=   ((I2CSlaveBuffer[2]>>4) & 0x03);

    dom=    ((I2CSlaveBuffer[4]>>0) & 0x0F);
    dom10=  ((I2CSlaveBuffer[4]>>4) & 0x03);
    mon=    ((I2CSlaveBuffer[5]>>0) & 0x0F);
    mon10=  ((I2CSlaveBuffer[5]>>4) & 0x01);
    year=   ((I2CSlaveBuffer[6]>>0) & 0x0F);
    year10= ((I2CSlaveBuffer[6]>>4) & 0x0F);

    /*
    rtc_dateArr[0]= rtc_date10;
    rtc_dateArr[1]= rtc_date1;
    rtc_dateArr[2]= rtc_month10;
    rtc_dateArr[3]= rtc_month1;
    rtc_dateArr[4]= rtc_year10;
    rtc_dateArr[5]= rtc_year1;
    */
    
/*
    rtc_date10= inputArr[0];
    rtc_date1=  inputArr[1];
    rtc_month10=    inputArr[2];
    rtc_month1= inputArr[3];
    rtc_year10= inputArr[4];
    rtc_year1=  inputArr[5];
*/

    UART_printf("%d%d:%d%d:%d%d\n\r", hr10, hr, min10, min, sec10, sec);
    UART_printf("%d%d - %d%d - %d%d\n\r", dom10, dom, mon10, mon, year10, year);
        
    return I2CSlaveBuffer[0];
}

void StartOsc(void){
    
    I2CWriteLength = 2;
    I2CReadLength = 4;
    I2CMasterBuffer[0] = ADDR_RTC_SRAM; 
    I2CMasterBuffer[1] = 0x00; // Start read register
    I2CMasterBuffer[2] = ADDR_RTC_SRAM | RD_BIT;
    // Put if statement here!
    I2CEngine();
    
    //scandal_naive_delay(100000);

    //If the START OSCILLATOR bit is not set, set it:
    if((I2CSlaveBuffer[0] & 0x80) == 0){
      I2CWriteLength = 3;
      I2CReadLength = 0;
      I2CMasterBuffer[0] = ADDR_RTC_SRAM;
      I2CMasterBuffer[1] = 0x00; // Start read register
      I2CMasterBuffer[2] =  I2CSlaveBuffer[0] | 0x80; //Enables the START OSCILLATOR bit so the clock ticks
      I2CEngine();
    }


    //If the VBATEN (Enable battery if power goes out)
    if((I2CSlaveBuffer[3] & 0x08) == 0){
      I2CWriteLength = 3;
      I2CReadLength = 0;
      I2CMasterBuffer[0] = ADDR_RTC_SRAM;
      I2CMasterBuffer[1] = 0x03; // Start read register
      I2CMasterBuffer[2] =  I2CSlaveBuffer[3] | 0x08; //Enables the VBATEN so
      I2CEngine();
    }

    
}

int SetTime(char* TimeArray){
    
    int sec, sec10; //Second in minute
    int min, min10; //Minute of hour
    int hr, hr10;   //Hour of day

    hr10=	TimeArray[0];
    hr=	TimeArray[1];
    min10=	TimeArray[2];
    min=	TimeArray[3];
    sec10=	TimeArray[4];
    sec=	TimeArray[5];
    
    I2CWriteLength = 5;
    I2CReadLength = 0;
    I2CMasterBuffer[0] = ADDR_RTC_SRAM;
    I2CMasterBuffer[1] = 0x00; // Start read register
    I2CMasterBuffer[2] = ENABLE_OSC | (sec10 << 4) | sec;
    I2CMasterBuffer[3] = (min10 << 4) | min;
    I2CMasterBuffer[4] = (hr10 << 4) | hr;
    
    I2CEngine();

    return 0;
    
}

int SetDate(char* DateArray){
    
    int dom, dom10;     //Date of month
    int mon, mon10;     //Month of year
    int year, year10;   //Year in XX format

    dom10=	DateArray[0];
    dom=	DateArray[1];
    mon10=	DateArray[2];
    mon=	DateArray[3];
    year10=	DateArray[4];
    year=	DateArray[5];
    
    I2CWriteLength = 5;
    I2CReadLength = 0;
    I2CMasterBuffer[0] = ADDR_RTC_SRAM;
    I2CMasterBuffer[1] = 0x04; // Start read register
    I2CMasterBuffer[2] = (dom10 << 4) | dom;
    I2CMasterBuffer[3] = (mon10 << 4) | mon;
    I2CMasterBuffer[4] = (year10 << 4) | year;
    
    I2CEngine();

    return 0;
    
}

void SmartUpdate(char inputArr[], uint32_t inputVal, uint8_t type){

    static uint8_t gps_date1, gps_date10, gps_month1, gps_month10, gps_year1, gps_year10;
    static uint8_t gps_sec, gps_sec10, gps_min, gps_min10, gps_hr, gps_hr10;
    static uint8_t rtc_date1, rtc_date10, rtc_month1, rtc_month10, rtc_year1, rtc_year10;
    static uint8_t rtc_sec, rtc_sec10, rtc_min, rtc_min10, rtc_hr, rtc_hr10;
    
    uint8_t rtc_Valid=0, gps_Valid=0;

    if(type==0){
      
    }
    
    //GPS date input
    if(type==1){
      gps_date10=   inputArr[0];
      gps_date1=    inputArr[1];
      gps_month10=  inputArr[2];
      gps_month1=   inputArr[3];
      gps_year10=   inputArr[4];
      gps_year1=    inputArr[5];
    }

    //GPS time input
    if(type==2){
      gps_hr10=	inputArr[0];
      gps_hr=	inputArr[1];
      gps_min10=	inputArr[2];
      gps_min=	inputArr[3];
      gps_sec10=	inputArr[4];
      gps_sec=	inputArr[5];
    }

    if(type==3){
      rtc_date10=   inputArr[0];
      rtc_date1=    inputArr[1];
      rtc_month10=  inputArr[2];
      rtc_month1=   inputArr[3];
      rtc_year10=   inputArr[4];
      rtc_year1=    inputArr[5];
    }

    /*If GPS locked and valid, time and date to the RTC
    The other statements are to ensure the data is reasonable (year > 2003)
    and that a new minute is not just about to come as it may cause errors on the RTC
    depending on when the writes happen. This is to avoid having to stop and restart the oscillator
    

    */
    if((type==5) && ((gps_year10*10 + gps_year1) > 3) && ((gps_sec10*10 + gps_sec) < 55) ){
    
	  
    }


    if((rtc_year1 + rtc_year10 * 10) > 2){
      rtc_Valid = 1;
    }

    
}
