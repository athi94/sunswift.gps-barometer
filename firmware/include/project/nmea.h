#if defined(msp430f149)
#include <sys/inttypes.h>
#else
#if defined(lpc11c14)
#include <scandal/types.h>
#include <arch/types.h>
#endif
#endif

#define NMEA_BUFFER_SIZE     128

#define TIME(h,m,s) (((((h)*60) + (m))*60) + (s));
#define HOURS(t) ((t)/(60*60))
#define MINUTES(t) (((t)/60)%60)
#define SECONDS(t) ((t)%60)

/* These macros calculate the components of the number based on the number of 
 * milliminutes, as opposed to converting from minutes to degrees
 */
/*
#define SIG(x) ((x) > 0 ? 1 : -1)
#define SIG_NS(x) ((x) > 0 ? 'N' : 'S')
#define SIG_EW(x) ((x) > 0 ? 'E' : 'W')
#define ABS(x) ((x) > 0 ? (x) : -(x))
#define DEG(mm) (ABS(mm)/(1000*60))
#define MIN(mm) ((ABS(mm)/1000)%60)
#define SEC(mm) (((ABS(mm)*60)/1000)%60)
#define MSEC(mm) ((ABS(mm)*60)%1000)
*/

typedef struct {
	uint32_t time;	/* ((hours*60 + minutes)*60 + seconds) * 1000 */
	int32_t lat;	/* latitude - milliminutes */
	int32_t lng;	/* longitude - milliminutes */
	int32_t alt;	/* altitude - millimetres */
} gps_point;

int validate_nmea(char* input);
int read_line(char* buf, char** output, int tokens);
int parse_time(char* time, uint32_t* result);
int parse_latitude(char* position, char* direction, int32_t* result);
int parse_longitude(char* position, char* direction, int32_t* result);
int parse_altitude(char* alt, char* units, int32_t* result);
int parse_date_gprmc(char* date, uint32_t* result);
int parse_time_gprmc(char* time, uint32_t* result);
int parse_msg_gga(char* buf, gps_point* p);
char *get_gga_time_string(void);
int nmea_read(char* buf, int len);
int nmea_readline();
void nmea_readbuf(char* buf);


typedef struct{
        uint32_t time;
        uint32_t date;
        uint32_t speed;       /* speed */ 
} gps_speed;

int parse_msg_rmc(char* buf, gps_speed* p);
int parse_speed(char* speed, uint32_t* result);
