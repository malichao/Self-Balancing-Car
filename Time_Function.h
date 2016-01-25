#ifndef Time_Function_h
#define Time_Function_h

#define BUSCLOCK80M

static unsigned long time_ms=0;
static unsigned long time_ms_mod=0;
static unsigned long time_us=0;

void setbusclock_80M(void);
void Dly_us(unsigned int);
void Dly_ms(unsigned int);
void PIT_Init(void);
unsigned long micros(void);
unsigned long millis(void);

static unsigned signa=0;

#endif