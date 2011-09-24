#include <sys/types.h>

void InitRTC(void);
int ReadTime(void);
void StartOsc(void);

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