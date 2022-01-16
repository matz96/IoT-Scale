#ifndef APP_MAIN
#define APP_MAIN

#define PWM_MAX_VAL 1800
#define PWM_MIN_VAL 0

#define ADC_MAX_VALUE 4096

#define KP 25
#define KI 30
#define LOW 1820
#define HIGH 2600
#define BIAS 0
#define TS 0.0002

#define CONVERSION_GR_OZ  0.035274

#define BTN_TARA 4
#define BTN_UINT 8

#define START_DIST_OFFEST 10
#define RPZERO_ADDR  (0x03<<1)

void app_main(void);


#endif
