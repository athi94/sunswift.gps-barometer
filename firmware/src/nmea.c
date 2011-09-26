#include <sys/types.h>

#include <scandal/uart.h>
#include <project/nmea.h>

#include <scandal/leds.h>


//extern uint8_t buffer_ready;
//extern uint8_t buf_flag;
//extern char buf0[NMEA_BUFFER_SIZE];
//extern char buf1[NMEA_BUFFER_SIZE];

static char *gga_time_array=0;
static char *rmc_date_array=0;

int validate_nmea(char* input){
	char realsum = 0;
	char checksum;
	
	if(*input=='$')
		input++;

	while(*input!='*') {
		if(*input=='\0') {
			return 1;	/* no checksum on message */
		}
		realsum^=*input;
		input++;
	}

	*input='\0';	/* remove checksum from string */

	if(*++input!='\0') {
		if(*input>='0' && *input<='9') {
			checksum = (*input - '0') << 4;
		} else if(*input>='A' && *input<='F') {
			checksum = (*input - 'A' + 0xA) << 4;
		} else {
			return -1;
		}
	} else {
		return -1;
	}

	if(*++input!='\0') {
		if(*input>='0' && *input<='9') {
			checksum |= (*input - '0');
		} else if(*input>='A' && *input<='F') {
			checksum |= *input - 'A' + 0xA;
		} else {
			return -1;
		}
	} else {
		return -1;
	}

	if(checksum==realsum) {
		return 0;
	}

	return -1;
}

/*
 * - buf will be changed and must be writable!
 * - data before first ',' is dropped.
 * - '\n' is acceptible within a token, so trim first
 * - anything after <tokens> ','s will be in last token
 */
int read_line(char* buf, char** output, int tokens){
	uint32_t t;

	for(t=0; t<tokens; t++){
		while(*buf!=',')
			if(*buf=='\0')
				return -1;
			else
				buf++;
		*buf='\0';
		output[t]=++buf;
	}

	return 0;
}

int parse_time(char* time, uint32_t* result){
        uint32_t i;
        uint32_t r;
	  static char t[6];

	for(i=0; i<6; i++){
		if(*time>='0' && *time<='9')
			t[i]=*time-'0';
		else
			return -1;
		time++;
	}

    /*Stores the location of this static time array
     * such that it can be grabbed for use by other things */
    gga_time_array=&t[0];


	r =(uint32_t)t[0]*10*60*60;
	r+=(uint32_t)t[1]*60*60;
	r+=(uint32_t)t[2]*10*60;
	r+=(uint32_t)t[3]*60;
	r+=(uint32_t)t[4]*10;
	r+=(uint32_t)t[5];

	r*=(uint32_t)1000; /* convert to milliseconds */

	*result = r;

	return 0;
}

int parse_time_gprmc(char* time, uint32_t* result){
        int i;
        uint32_t r;
	  char t[10];

	for(i=0; i<10; i++){
		if(*time>='0' && *time<='9')
			t[i]=*time-'0';
		else
            if(*time != '.')
                return -1;
		time++;
	}
	
	r = 0; 
	r = t[0]; 
	r *= 10;   r += t[1]; 
	r *= 6;   r += t[2]; 
	r *= 10;   r += t[3]; 
	r *= 6;   r += t[4]; 
	r *= 10;   r += t[5]; 
	r *= 10;   r += t[6]; 
	r *= 10;   r += t[7]; 
	r *= 10;   r += t[8]; 
	
	*result = r;

	return 0;
}


/* This code converts a DDMMYY format date (with YY being YYYY - 2000) 
    to the number of days since the UNIX epoch - 1 Jan 1970 */ 
#define YEAR_2000_DAYS_SINCE_EPOCH 10957

const int _ytab[2][12] = {
  { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 },
  { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }
};

uint32_t 
days_since_epoch(int day, int month, int year){
/* Year is in 20XX format, where year is the XX */ 
    int ly, i; 
    uint32_t days; 
    int hundred_year_count, four_year_count, four_hundred_year_count; 
  
    days = 0; 
    days = YEAR_2000_DAYS_SINCE_EPOCH;
    
    four_hundred_year_count = 0; 
    hundred_year_count = 0; 
    four_year_count = 0; 
    while(year-- > 0){
        if((four_hundred_year_count--) == 0){
            days += 366; 
            four_hundred_year_count = 399;
            hundred_year_count = 99; 
            four_year_count = 3;
        }else if((hundred_year_count--) == 0){ 
            days += 365;
            hundred_year_count = 99;
            four_year_count = 3; 
        }else if((four_year_count--) == 0){
            days += 366; 
            four_year_count = 3; 
        }else
            days += 365;
    }

    /* Check to see if this is a leap year */ 
    if((four_hundred_year_count == 0) || 
        ((four_year_count == 0) && (hundred_year_count != 0)))
        ly = 1; 
    else 
        ly = 0;
         
    days += (day - 1); 
    for(i=0; i<(month - 1); i++){
        days += _ytab[ly][i];
    }
    
    return days; 
}


int 
parse_date_gprmc(char* date, uint32_t* result){
    int i;
    uint32_t r;
    static char t[6];

    for(i=0; i<6; i++){
	  if(*date>='0' && *date<='9')
		    t[i]=*date-'0';
	  else
	  return -1;
	  date++;
    }
    
    rmc_date_array=&t[0];
    
    r = days_since_epoch(   t[0] * 10 + t[1], 
                            t[2] * 10 + t[3], 
                            t[4] * 10 + t[5] );
    
    *result = r;

    return 0;
}


int parse_latitude(char* position, char* direction, int32_t* result){
	int32_t d, m, mm;
	int32_t w=0, f=0;

	/* read digits of degree + min */
	while(*position!='.'){
		if(*position>='0' && *position<='9')
			w = (w*10) + (*position - '0');
		else
			return -1;
		position++;
	}

	/* read fraction (1, 2, 3 dp) */
	position++;
	if(*position>='0' && *position<='9'){
		f += (*position - '0') * 100;

		position++;
		if(*position>='0' && *position<='9'){
			f += (*position - '0') * 10;

			position++;
			if(*position>='0' && *position<='9')
				f += *position - '0';
		}
	}

	d = w/100;
	m = (w%100) + (d*60);
	mm = (m*1000) + f;
	
	if(*direction=='N')
		*result = mm;
	else if(*direction=='S')
		*result = -mm;
	else
		return -1;

	return 0;
}

int parse_longitude(char* position, char* direction, int32_t* result){
	int32_t d, m, mm;
	int32_t w=0, f=0;

	/* read digits of degree + min */
	while(*position!='.'){
		if(*position>='0' && *position<='9')
			w = (w*10) + (*position - '0');
		else
			return -1;
		position++;
	}

	/* read fraction (1, 2, 3 dp) */
	position++;
	if(*position>='0' && *position<='9'){
		f += (*position - '0') * 100;

		position++;
		if(*position>='0' && *position<='9'){
			f += (*position - '0') * 10;

			position++;
			if(*position>='0' && *position<='9')
				f += *position - '0';
		}
	}

	d = w/100;
	m = (w%100) + (d*60);
	mm = (m*1000) + f;
	
	if(*direction=='E')
		*result = mm;
	else if(*direction=='W')
		*result = -mm;
	else
		return -1;

	return 0;
}

int parse_altitude(char* alt, char* units, int32_t* result){
	int32_t m=0;
	int32_t cm = 0;

	if(*units!='M')
		return -1;

	/* do integer part */
	while(*alt!='.'){
		if(*alt>='0' && *alt<='9')
			m = (m*10) + (*alt - '0');
		else
			return -1;
		alt++;
	}

	/* do 2 decimal places */
	if((alt++, *alt>='0' && *alt<='9')){
		cm += (*alt - '0') * 10;

		if((alt++, *alt>='0' && *alt<='9'))
			cm += (*alt - '0');
	}

	*result = ((m*100) + cm)*10; /* millimetres */

	return 0;
}

int parse_speed(char* speed, uint32_t* result){
	uint32_t m=0;
	uint32_t cm = 0;

	/* do integer part */
	while(*speed!='.'){
		if(*speed>='0' && *speed<='9')
			m = (m*10) + (*speed - '0');
		else
			return -1;
		speed++;
	}
        
	speed++; 
	if( *speed>='0' && *speed<='9' ){
	  cm += (*speed - '0') * 10;
	  
	  speed++;
	  if(*speed >= '0' && *speed<='9')
	    cm += (*speed - '0');
	}

	*result = (m+(cm/100.0))*(1852.0/1000.0); /* metres per second */
	return 0;
}


#define MSG_GGA_TOKENS 14
int parse_msg_gga(char* buf, gps_point* p){
	char* token[MSG_GGA_TOKENS];
	
	if(read_line(buf, token, MSG_GGA_TOKENS))
		goto err;

	/* GPGGA token 6 contains fix data */
	if(!(*token[5]=='1' || *token[5]=='2'))
		return -2;

	if(parse_time(token[0], &p->time)) {
		goto err;
	}
	
	if(parse_latitude(token[1], token[2], &p->lat))
		goto err;

	if(parse_longitude(token[3], token[4], &p->lng))
		goto err;

	if(parse_altitude(token[8], token[9], &p->alt))
		goto err;
	//gga_time_string = token[0];
	//UART_printf("Tokens: %d:%d %d:%d %d:%d\r\n", gga_time_string++, gga_time_string++, gga_time_string++, gga_time_string++, gga_time_string++, gga_time_string);
	//gga_time_string = token[0];
	
	return 0;
err:
	return -1;
}

char *get_gga_time_array(void) {
    if (gga_time_array[0] != '0')
	  return gga_time_array;
    else
	  return NULL;
}

char *get_rmc_date_array(void) {
    if (rmc_date_array[0] != '0')
	  return rmc_date_array;
    else
	  return NULL;
}

#define MSG_RMC_TOKENS 11
int parse_msg_rmc(char* buf, gps_speed* p){
	char* token[MSG_RMC_TOKENS];

	if(read_line(buf, token, MSG_RMC_TOKENS))
		goto err;

	if(parse_time_gprmc(token[0], &p->time))
		goto err;

	if(parse_date_gprmc(token[8], &p->date))
		goto err; 

	if(parse_speed(token[6], &p->speed))
		goto err;

	return 0;
err:
	
	return -1;
}

#if 0
/* dont forget to free the result! */
int nmea_readline(char* buf){
	int i = 0;

#if NMEA_BUFFER_SIZE != 128
#error Wrong buffer size!
#endif

	do{
		for(i=0; i<NMEA_BUFFER_SIZE; i++){
		  buf[i] = UART_ReceiveByte();

			if(buf[0] != '$')
				return -1;

		    if(buf[i]=='\r' || buf[i]=='\n' || buf[i]=='\0'){
		      buf[i]='\0';
		      break;
		    }
		}
		buf[NMEA_BUFFER_SIZE]='\0';
	} while(buf[0]=='\0');
	
	return 0;
}



/*Copy from recieve buffers into nmea processing buffer*/
void nmea_readbuf(char* buf) {
	int i = 0;

	if(buf_flag == BUF0){
		while(i < NMEA_BUFFER_SIZE){
			buf[i] = buf0[i];
			i++;
		}
	} else{
		while(i < NMEA_BUFFER_SIZE){
			buf[i] = buf1[i];
			i++;
		}
	}
	buffer_ready = 0;
}
 
#endif
