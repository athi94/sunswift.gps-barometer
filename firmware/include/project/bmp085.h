#include <sys/types.h>

extern volatile uint32_t I2CCount;
extern volatile uint8_t I2CMasterBuffer[BUFSIZE];
extern volatile uint8_t I2CSlaveBuffer[BUFSIZE];
extern volatile uint32_t I2CMasterState;
extern volatile uint32_t I2CReadLength, I2CWriteLength;

typedef struct bmp085_cal_struct *Bmp085_cal;
struct bmp085_cal_struct {
  short AC1;
  short AC2;
  short AC3;
  unsigned short AC4;
  unsigned short AC5;
  unsigned short AC6;
  short B1;
  short B2;
  short MB;
  short MC;
  short MD;
};
//Over Sampling Setting
#define 0SS 		0
//Refer to page ten of BMP085 datsheet
//0: Ultra low power
//1: Standard
//2: High resolution
//3: Ultra High resolution
#define BMP_MODULE_ADDRESS 	0xEE
#define BMP_WRITE 		0x00
#define BMP_READ		0x01

#define TEMP_READ 		0x2E
#define PRES_READ 		(0x34 + (OSS << 6))

#define CTL_REG 		0xF4
//Address refers to msb, lsb is the next addres
#define DATA_REG		0xF6
#define AC1_REG			0xAA
#define AC2_REG			0xAC
#define AC3_REG			0xAE
#define AC4_REG			0xB0
#define AC5_REG			0xB2
#define AC6_REG			0xB4
#define B1_REG			0xB6
#define B2_REG			0xB8
#define MB_REG			0xBA
#define MC_REG			0xBC
#define MD_REG			0xBE

#define PRESSURE_SEALEVEL 101325 //Pa

Bmp085_cal readCalibrationValues(void);
long bmp085ReadUT(void)
long bmp085ReadUP(void)

long bmp085Getb5((long ut,Bmp085_cal barometerCal);
long bmp085GetTemperature(long ut,long b5);
long bmp085GetPressure(long up, Bmp085_cal barometerCal, long b5);
long bmp085GetAltitude(long pres);

unsigned char bmp085ReadByte(unsigned char address);
short bmp085ReadShort(unsigned char address);
void bmp085WriteByte(unsigned char address, unsigned char value);


