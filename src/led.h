extern int LedState;

#define LED_OFF 0
#define LED_ON 1
#define WARNING_TIME 10000

bool Led_Init(void);
void Led_Standby(void);
void Led_Warning(void);
void Led_Recording(void);
void Led_Finish(void);
void Led_Check(int bat_state, bool flag);
void Led_preallocate(void);