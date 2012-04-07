#include <sys/types.h>

void InitRTC(void);
int ReadTime(void);
void StartOsc(void);
uint64_t getRTCTimeSecond(void);
int getRTCDay(void);
int getRTCMonth(void);
int getRTCYear(void);

/*
typedef struct HumanTime{
    int sec;
    int min;
    int hr;
    int dom;
    int mon;
    int year;
}
*/