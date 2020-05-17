#include "pic24_all.h"
#include <stdio.h>
#include <stdlib.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//LED configuration
#define CONFIG_LED1()    CONFIG_RB15_AS_DIG_OUTPUT()
#define LED1             (_LATB15)

// LED Blinking
void BlinkLED (uint8_t u8_a) {
    uint8_t u8_toggle;
    for (u8_toggle = 0; u8_toggle < (u8_a * 2); u8_toggle++) {
        LED1 = !LED1;
        DELAY_MS(300);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//LCD configuration
#define RS_HIGH()        (_LATB12 = 1)
#define RS_LOW()         (_LATB12 = 0)
#define CONFIG_RS()      CONFIG_RB12_AS_DIG_OUTPUT()

#define RW_HIGH()        (_LATB13 = 1)
#define RW_LOW()         (_LATB13 = 0)
#define CONFIG_RW()      CONFIG_RB13_AS_DIG_OUTPUT()

#define E_HIGH()         (_LATB14 = 1)
#define E_LOW()          (_LATB14 = 0)
#define CONFIG_E()       CONFIG_RB14_AS_DIG_OUTPUT()

#define LCD4O          (_LATB6)
#define LCD5O          (_LATB7)
#define LCD6O          (_LATB8)
#define LCD7O          (_LATB9)
#define LCD7I          (_RB9)

#define CONFIG_LCD4_AS_INPUT() CONFIG_RB6_AS_DIG_INPUT()
#define CONFIG_LCD5_AS_INPUT() CONFIG_RB7_AS_DIG_INPUT()
#define CONFIG_LCD6_AS_INPUT() CONFIG_RB8_AS_DIG_INPUT()
#define CONFIG_LCD7_AS_INPUT() CONFIG_RB9_AS_DIG_INPUT()

#define CONFIG_LCD4_AS_OUTPUT() CONFIG_RB6_AS_DIG_OUTPUT()
#define CONFIG_LCD5_AS_OUTPUT() CONFIG_RB7_AS_DIG_OUTPUT()
#define CONFIG_LCD6_AS_OUTPUT() CONFIG_RB8_AS_DIG_OUTPUT()
#define CONFIG_LCD7_AS_OUTPUT() CONFIG_RB9_AS_DIG_OUTPUT()

#define GET_BUSY_FLAG()  (LCD7I)
/**
 Functions above this line must be redefined for
 your particular PICmicro-to-LCD interface
*/

//Configure 4-bit data bus for output
void configBusAsOutLCD(void) {
  RW_LOW();                  //RW=0 to stop LCD from driving pins
  CONFIG_LCD4_AS_OUTPUT();   //D4
  CONFIG_LCD5_AS_OUTPUT();   //D5
  CONFIG_LCD6_AS_OUTPUT();   //D6
  CONFIG_LCD7_AS_OUTPUT();   //D7
}

//Configure 4-bit data bus for input
void configBusAsInLCD(void) {
  CONFIG_LCD4_AS_INPUT();   //D4
  CONFIG_LCD5_AS_INPUT();   //D5
  CONFIG_LCD6_AS_INPUT();   //D6
  CONFIG_LCD7_AS_INPUT();   //D7
  RW_HIGH();                // R/W = 1, for read
}

//Output lower 4-bits of u8_c to LCD data lines
void outputToBusLCD(uint8_t u8_c) {
  LCD4O = u8_c & 0x01;          //D4
  LCD5O = (u8_c >> 1)& 0x01;    //D5
  LCD6O = (u8_c >> 2)& 0x01;    //D6
  LCD7O = (u8_c >> 3)& 0x01;    //D7
}

//Configure the control lines for the LCD
void configControlLCD(void) {
  CONFIG_RS();     //RS
  CONFIG_RW();     //RW
  CONFIG_E();      //E
  RW_LOW();
  E_LOW();
  RS_LOW();
}

//Pulse the E clock, 1 us delay around edges for
//setup/hold times
void pulseE(void) {
  DELAY_US(1);
  E_HIGH();
  DELAY_US(1);
  E_LOW();
  DELAY_US(1);
}

/* Write a byte (u8_Cmd) to the LCD.
u8_DataFlag is '1' if data byte, '0' if command byte
u8_CheckBusy is '1' if must poll busy bit before write, else simply delay before write
u8_Send8Bits is '1' if must send all 8 bits, else send only upper 4-bits
*/
void writeLCD(uint8_t u8_Cmd, uint8_t u8_DataFlag,
              uint8_t u8_CheckBusy, uint8_t u8_Send8Bits) {

  uint8_t u8_BusyFlag;
  uint8_t u8_wdtState;
  if (u8_CheckBusy) {
    RS_LOW();            //RS = 0 to check busy
    // check busy
    configBusAsInLCD();  //set data pins all inputs
    u8_wdtState = _SWDTEN;  //save WDT enable state
    CLRWDT();          //clear the WDT timer
    _SWDTEN = 1;            //enable WDT to escape infinite wait
        do {
          E_HIGH();
          DELAY_US(1);  // read upper 4 bits
          u8_BusyFlag = GET_BUSY_FLAG();
          E_LOW();
          DELAY_US(1);
          pulseE();              //pulse again for lower 4-bits
        } while (u8_BusyFlag);
    _SWDTEN = u8_wdtState;   //restore WDT enable state
  }
  else {
    DELAY_MS(10); // don't use busy, just delay
  }
  configBusAsOutLCD();
  if (u8_DataFlag) RS_HIGH();   // RS=1, data byte
  else  RS_LOW();             // RS=0, command byte
  outputToBusLCD(u8_Cmd >> 4);  // send upper 4 bits
  pulseE();
  if (u8_Send8Bits) {
    outputToBusLCD(u8_Cmd);     // send lower 4 bits
    pulseE();
  }
}

//These definitions are for a Hantronix 20x4 LCD
//#define GOTO_LINE1() writeLCD(0x80,0,1,1)
//#define GOTO_LINE2() writeLCD(0xC0,0,1,1)
//#define GOTO_LINE3() writeLCD(0x94,0,1,1)
//#define GOTO_LINE4() writeLCD(0xD4,0,1,1)

// Initialize the LCD, modify to suit your application and LCD
void initLCD() {
  DELAY_MS(50);          //wait for device to settle
  writeLCD(0x20,0,0,0); // 4 bit interface
  writeLCD(0x28,0,0,1); // 2 line display, 5x7 font
  writeLCD(0x28,0,0,1); // repeat
  writeLCD(0x06,0,0,1); // enable display
  writeLCD(0x0C,0,0,1); // turn display on; cursor, blink is off
  writeLCD(0x01,0,0,1); // clear display, move cursor to home
  DELAY_MS(3);
}

//Output a string to the LCD
void outStringLCD(char *psz_s) {
  while (*psz_s) {
    writeLCD(*psz_s,1,1,1);
    psz_s++;
  }
}

//LCD custom character
void LCD_build (unsigned char location, unsigned char *ptr){
    uint8_t i;
    if (location < 8) {
        writeLCD((0x40 + (location * 8)),0,0,1);
        for( i = 0; i < 8; i++)
            writeLCD(ptr[i],1,1,1);
    }
}

//Bar data in LCD CGRAM
unsigned char bar1[8] = {0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10};
unsigned char bar2[8] = {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18};
unsigned char bar3[8] = {0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C};
unsigned char bar4[8] = {0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E};
unsigned char bar5[8] = {0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//KEYPAD configuration
#define C0 _RA2
#define C1 _RA3
#define C2 _RB5

static inline void CONFIG_COLUMN() {
  CONFIG_RA2_AS_DIG_INPUT();
  ENABLE_RA2_PULLUP();
  CONFIG_RA3_AS_DIG_INPUT();
  ENABLE_RA3_PULLUP();
  CONFIG_RB5_AS_DIG_INPUT();
  ENABLE_RB5_PULLUP();
}

#define R0 _LATB9
#define R1 _LATB8
#define R2 _LATB7
#define R3 _LATB6

#define CONFIG_R0_DIG_OUTPUT() CONFIG_RB9_AS_DIG_OUTPUT()
#define CONFIG_R1_DIG_OUTPUT() CONFIG_RB8_AS_DIG_OUTPUT()
#define CONFIG_R2_DIG_OUTPUT() CONFIG_RB7_AS_DIG_OUTPUT()
#define CONFIG_R3_DIG_OUTPUT() CONFIG_RB6_AS_DIG_OUTPUT()
 
void CONFIG_ROW() {
  CONFIG_R0_DIG_OUTPUT();
  CONFIG_R1_DIG_OUTPUT();
  CONFIG_R2_DIG_OUTPUT();
  CONFIG_R3_DIG_OUTPUT();
}

static inline void DRIVE_ROW_LOW() {
  R0 = 0;
  R1 = 0;
  R2 = 0;
  R3 = 0;
}

static inline void DRIVE_ROW_HIGH() {
  R0 = 1;
  R1 = 1;
  R2 = 1;
  R3 = 1;
}

void configKeypad(void) {
  CONFIG_ROW();
  DRIVE_ROW_LOW();
  CONFIG_COLUMN();
  DELAY_US(1);     //wait for pullups to stabilize inputs
}

//drive one row low
void setOneRowLow(uint8_t u8_x) {
  switch (u8_x) {
    case 0:
      R0 = 0;
      R1 = 1;
      R2 = 1;
      R3 = 1;
      break;
    case 1:
      R0 = 1;
      R1 = 0;
      R2 = 1;
      R3 = 1;
      break;
      case 2:
      R0 = 1;
      R1 = 1;
      R2 = 0;
      R3 = 1;
      break;
    default:
      R0 = 1;
      R1 = 1;
      R2 = 1;
      R3 = 0;
  }
}
#define NUM_ROWS 4
#define NUM_COLS 3
const uint8_t au8_keyTable[NUM_ROWS][NUM_COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

#define KEY_PRESSED() (!C0 || !C1 || !C2)   //any low
#define KEY_RELEASED() (C0 && C1 && C2)  //all high

uint8_t doKeyScan(void) {
  uint8_t u8_row, u8_col;
  //determine column
  if (!C0) u8_col = 0;
  else if (!C1) u8_col = 1;
  else if (!C2) u8_col = 2;
  //else if (!C3) u8_col = 3;
  else return('E'); //error
  //determine row
  for (u8_row = 0; u8_row < NUM_ROWS; u8_row++) {
    setOneRowLow(u8_row); //enable one row low
    if (KEY_PRESSED()) {
      DRIVE_ROW_LOW(); //return rows to driving low
      return(au8_keyTable[u8_row][u8_col]);
    }
  }
  DRIVE_ROW_LOW(); //return rows to driving low
  return('E'); //error
}

typedef enum  {
  STATE_WAIT_FOR_PRESS = 0,
  STATE_WAIT_FOR_PRESS2,
  STATE_WAIT_FOR_RELEASE,
} ISRSTATE;

ISRSTATE e_isrState = STATE_WAIT_FOR_PRESS;
volatile uint8_t u8_newKey = 0;

//Interrupt Service Routine for Timer3
void _ISR _T3Interrupt (void) {
  switch (e_isrState) {
    case STATE_WAIT_FOR_PRESS:
      if (KEY_PRESSED() && (u8_newKey == 0)) {
        //ensure that key is sampled low for two consecutive interrupt periods
        e_isrState = STATE_WAIT_FOR_PRESS2;
      }
      break;
    case STATE_WAIT_FOR_PRESS2:
      if (KEY_PRESSED()) {
        // a key is ready
        u8_newKey = doKeyScan();
        e_isrState = STATE_WAIT_FOR_RELEASE;
      } 
      else e_isrState = STATE_WAIT_FOR_PRESS;
      break;

    case STATE_WAIT_FOR_RELEASE:
      //keypad released
      if (KEY_RELEASED()) {
        e_isrState = STATE_WAIT_FOR_PRESS;
      }
      break;

    default:
      e_isrState = STATE_WAIT_FOR_PRESS;
      break;
  }
  
  _T3IF = 0; //clear the timer interrupt bit
}

#define ISR_PERIOD     15      // in ms

void  configTimer3(void) {
  //ensure that Timer2,3 configured as separate timers.
  T2CONbits.T32 = 0;     // 32-bit mode off
  //T3CON set like this for documentation purposes.
  //could be replaced by T3CON = 0x0020
  T3CON = T3_OFF | T3_IDLE_CON | T3_GATE_OFF
          | T3_SOURCE_INT
          | T3_PS_1_64 ;  //results in T3CON= 0x0020
  PR3 = msToU16Ticks(ISR_PERIOD, getTimerPrescale(T3CONbits)) - 1;
  TMR3  = 0;                       //clear timer3 value
  _T3IF = 0;                       //clear interrupt flag
  _T3IP = 1;                       //choose a priority
  _T3IE = 1;                       //enable the interrupt
  //T3CONbits.TON = 1;               //turn on the timer
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _LPOSCEN
#error "This example only works with a device that has a secondary oscillator."
#endif
#ifndef _RTCSYNC
#error "This example only works with a device that has an RTCC module."
#endif

//RTCC configuration
typedef union _unionRTCC {
  struct {  //four 16 bit registers
    uint8_t yr;
    uint8_t null;
    uint8_t date;
    uint8_t month;
    uint8_t hour;
    uint8_t wday;
    uint8_t sec;
    uint8_t min;
  } u8;
  uint16_t regs[4];
} unionRTCC;

unionRTCC u_RTCC;

uint8_t getBCDvalue(char *sz_1) {
  char sz_buff[8];
  uint16_t u16_bin;
  uint8_t  u8_bcd;
  outString(sz_1);
  inStringEcho(sz_buff,7);
  sscanf(sz_buff,"%d", (int *)&u16_bin);
  u8_bcd = u16_bin/10;   //most significant digit
  u8_bcd = u8_bcd << 4;
  u8_bcd = u8_bcd | (u16_bin%10);
  return(u8_bcd);
}

void getDateFromUser(void) {
  u_RTCC.u8.yr = getBCDvalue("Enter year (0-99): ");
  u_RTCC.u8.month = getBCDvalue("Enter month (1-12): ");
  u_RTCC.u8.date = getBCDvalue("Enter day of month (1-31): ");
  u_RTCC.u8.wday = getBCDvalue("Enter week day (0-6): ");
  u_RTCC.u8.hour = getBCDvalue("Enter hour (0-23): ");
  u_RTCC.u8.min = getBCDvalue("Enter min (0-59): ");
  u_RTCC.u8.sec = getBCDvalue("Enter sec(0-59): ");
}

//set date
void setRTCC(void) {
  uint8_t u8_i;
  __builtin_write_RTCWEN();   //enable write to RTCC, sets RTCWEN
  RCFGCALbits.RTCEN = 0;      //disable the RTCC
  RCFGCALbits.RTCPTR = 3;     //set pointer reg to start
  for (u8_i=0; u8_i<4; u8_i++) RTCVAL = u_RTCC.regs[u8_i];
  RCFGCALbits.RTCEN = 1;     //Enable the RTCC
  RCFGCALbits.RTCWREN = 0;   //can clear without unlock
}

void readRTCC(void) {
  uint8_t u8_i;
  RCFGCALbits.RTCPTR = 3;     //set pointer reg to start
  for (u8_i=0; u8_i<4; u8_i++) u_RTCC.regs[u8_i] = RTCVAL;
}

void printRTCC(void) {
  printf ("\nday(wday)/mon/yr: %2x(%2x)/%2x/%2x, %02x:%02x:%02x \n",
          (uint16_t) u_RTCC.u8.date,(uint16_t) u_RTCC.u8.wday, (uint16_t) u_RTCC.u8.month,
          (uint16_t) u_RTCC.u8.yr, (uint16_t) u_RTCC.u8.hour, (uint16_t) u_RTCC.u8.min, (uint16_t) u_RTCC.u8.sec);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef PWM_PERIOD
#define PWM_PERIOD 20000  // desired period for SG90, in us
#endif

//Timer2 configuration
void  configTimer2(void) {
  T2CON = T2_OFF | T2_IDLE_CON | T2_GATE_OFF
          | T2_32BIT_MODE_OFF
          | T2_SOURCE_INT
          | T2_PS_1_64;
  PR2 = usToU16Ticks(PWM_PERIOD, getTimerPrescale(T2CONbits)) - 1;
  TMR2  = 0;       //clear timer2 value
  _T2IF = 0;
  _T2IP = 1;
  _T2IE = 1;    //enable the Timer2 interrupt
}

//Output Compare configuration
void configOutputCompare1(void) {
  T2CONbits.TON = 0;          //disable Timer when configuring Output compare
  CONFIG_OC1_TO_RP(RB1_RP);   //map OC1 to RB1
  OC1RS = 0;  //clear both registers
  OC1R = 0;
#ifdef OC1CON1
//turn on the compare toggle mode using Timer2
  OC1CON1 = OC_TIMER2_SRC |     //Timer2 source
            OC_PWM_CENTER_ALIGN;  //PWM
  OC1CON2 = OC_SYNCSEL_TIMER2;   //synchronize to timer2
#else
//older families, this PWM mode is compatible with center-aligned, OC1R=0
//as writes to OC1RS sets the pulse widith.
  OC1CON = OC_TIMER2_SRC |     //Timer2 source
           OC_PWM_FAULT_PIN_DISABLE;  //PWM, no fault detection
#endif
}

uint16_t min_pw, max_pw;
uint16_t u16_minPWTicks, u16_maxPWTicks;

//CW and CCW value setup
void setServoRange(void) {
  T2CONbits.TON = 0;  //turn off the timer2
  u16_minPWTicks = usToU16Ticks(min_pw, getTimerPrescale(T2CONbits));
  u16_maxPWTicks = usToU16Ticks(max_pw, getTimerPrescale(T2CONbits));
  T2CONbits.TON = 1;  //turn on the timer2
}

void _ISR _T2Interrupt(void) {
  uint32_t u32_temp;
  _T2IF = 0;    //clear the timer interrupt bit
  //update the PWM duty cycle from the ADC value
  u32_temp = ADC1BUF0;  //use 32-bit value for range
  //compute new pulse width that is 0 to 99% of PR2
  // ((max - min) * ADC)/4096 + min
  u32_temp = ((u32_temp * (u16_maxPWTicks - u16_minPWTicks))>> 12) + u16_minPWTicks;  // >>12 is same as divide/4096
  OC1RS = u32_temp;  //update pulse width value
  SET_SAMP_BIT_ADC1();      //start sampling and conversion
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Project functions */

/* Declare variables for project */
#define MIN_PW  600
#define MAX_PW  2400
#define CENTER  1500
#define RANGE   (MAX_PW - MIN_PW)

char lcdoutput[10];
uint8_t u8_eval;
uint32_t u32_pw;
uint16_t u16_ccw, u16_cw, u16_min, u16_max;
uint8_t u8_cell, u8_bar;
float f_cell, f_bar;
uint8_t i, j;

//CCW input number evaluation
uint8_t evaluateCCW (uint16_t num) {
    if (num < MIN_PW || num > CENTER) return 0;
    else return 1;
}

//CW input number evaluation
uint8_t evaluateCW (uint16_t num) {
    if (num < CENTER || num > MAX_PW) return 0;
    else return 1;
}

/* States for project */
typedef enum  {
  STATE_SELFCHECK,
  STATE_SELFCHECK1,
  STATE_OPTIONS_LCD_ON,
  STATE_OPTIONS,
  STATE_OPTIONS1_KEY_ON,
  STATE_OPTIONS1,
  STATE_SETUP_LCD_ON,
  STATE_SETUP,
  STATE_SETUP1_KEY_ON,
  STATE_SETUP1,
  STATE_SETUP_CW_CONF,
  STATE_SETUP_CW,
  STATE_SETUP_CW1_LCD_ON,
  STATE_SETUP_CW1,
  STATE_SETUP_CW2,
  STATE_SETUP_CW3,
  STATE_SETUP_CCW_CONF,
  STATE_SETUP_CCW,
  STATE_SETUP_CCW1_LCD_ON,
  STATE_SETUP_CCW1,
  STATE_SETUP_CCW2,
  STATE_SETUP_CCW3,
  STATE_RUN,
  STATE_RUN_CONF,
  STATE_RUN_STAT1,
  STATE_RUN_CALC,
  STATE_RUN_STAT2,
  STATE_RUN_KEY_ON,
  STATE_DOWNLOAD,
  STATE_DOWNLOAD1,
} state_t;

/* Project function */
void update_state(void) {
    static state_t e_state = STATE_SELFCHECK;
    switch (e_state) {
        case STATE_SELFCHECK :
            writeLCD(0x01,0,0,1);//clear screen and go to 1st line
            outStringLCD("** Self-Check **");
            DELAY_MS(500);
            writeLCD(0xC3,0,0,1);//clear screen and go to 2nd line and 4th location
            outStringLCD("System OK");
            e_state = STATE_SELFCHECK1;
            break;
        
        case STATE_SELFCHECK1 :
            min_pw = CENTER;
            max_pw = CENTER + 1;
            setServoRange();
            BlinkLED(1);
            DELAY_MS(1000);
            writeLCD(0x01,0,0,1);//clear screen and go to 1st line
            outStringLCD("Please select");
            writeLCD(0xC0,0,0,1);//goto second line
            outStringLCD("options");
            DELAY_MS(1500);
            e_state = STATE_OPTIONS;
            break;
            
        case STATE_OPTIONS_LCD_ON :
            T3CONbits.TON = 0;  //turn off the timer3 to disable Keypad
            e_state = STATE_OPTIONS;
            break;
            
        case STATE_OPTIONS :
            writeLCD(0x01,0,0,1);//clear screen and go to 1st line
            outStringLCD("1)SET UP   2)RUN");
            writeLCD(0xC0,0,0,1);//go to 2nd line
            outStringLCD("3)DOWNLOAD");
            DELAY_MS(200);
            e_state = STATE_OPTIONS1_KEY_ON;
            break;
            
        case STATE_OPTIONS1_KEY_ON :
            E_LOW();            //Disable LCD
            configKeypad();     //configure Keypad
            T3CONbits.TON = 1;  //turn on the timer3 to scan Keypad
            e_state = STATE_OPTIONS1;
            break;
            
        case STATE_OPTIONS1 :
            switch (u8_newKey) {
                case '1' :
                    e_state = STATE_SETUP_LCD_ON;
                    break;
                case '2' :
                    e_state = STATE_RUN;
                    break;
                case '3' :
                    e_state = STATE_DOWNLOAD;
                    break;
                default :
                    e_state = STATE_OPTIONS1;
            }
            break;

        case STATE_SETUP_LCD_ON :
            T3CONbits.TON = 0;  //turn off the timer3 to disable Keypad
            e_state = STATE_SETUP;
            break;
            
        case STATE_SETUP :
            writeLCD(0x01,0,0,1);//clear screen and go to 1st line
            outStringLCD("1)CW setup");
            writeLCD(0xC0,0,0,1);//go to 2nd line
            outStringLCD("2)CCW setup");
            DELAY_MS(200);
            e_state = STATE_SETUP1_KEY_ON;
            break;
            
        case STATE_SETUP1_KEY_ON :
            E_LOW();            //Disable LCD
            DRIVE_ROW_LOW();    //drive rows of Keypad low 
            T3CONbits.TON = 1;  //turn on the timer3 to scan Keypad
            e_state = STATE_SETUP1;
            break;
            
        case STATE_SETUP1 :
            switch (u8_newKey) {
                case '1' :
                    e_state = STATE_SETUP_CW_CONF;
                    break;
                case '2' :
                    e_state = STATE_SETUP_CCW_CONF;
                    break;
                default :
                    e_state = STATE_SETUP1;
            }
            break;
            
        case STATE_SETUP_CW_CONF :
            T3CONbits.TON = 0;  //turn off the timer3 to disable Keypad
            min_pw = CENTER;
            max_pw = MAX_PW + 1;
            setServoRange();
            e_state = STATE_SETUP_CW;
            break;
        
        case STATE_SETUP_CW :
            writeLCD(0x01,0,0,1); //clear screen and go to 1st line
            outStringLCD("Set up CW value");
            writeLCD(0xC0,0,0,1);//go to 2nd line
            outStringLCD("90deg to 180deg");
            DELAY_MS(1500);
            e_state = STATE_SETUP_CW1;
            break;
        
        case STATE_SETUP_CW1_LCD_ON :
            T3CONbits.TON = 0;  //turn off the timer3 to disable Keypad
            e_state = STATE_SETUP_CW1;
            break;
        
        case STATE_SETUP_CW1 :
            u16_max = ticksToUs(OC1RS, getTimerPrescale(T2CONbits));
            u16_cw = (u16_max - MIN_PW) / 10;
            writeLCD(0x01,0,0,1); //clear screen and go to 1st line
            sprintf(lcdoutput, "%d deg", u16_cw);
            outStringLCD(lcdoutput);
            writeLCD(0x88,0,0,1); //move cursor to the right
            sprintf(lcdoutput, "(%dus)", u16_max);
            outStringLCD(lcdoutput);
            writeLCD(0xC7,0,0,1);//go to 2nd line and 8th location
            outStringLCD("[#:ENTER]");
            e_state = STATE_SETUP_CW2;
            break;
            
        case STATE_SETUP_CW2 :
            E_LOW();            //Disable LCD
            DRIVE_ROW_LOW();    //drive rows of Keypad low 
            T3CONbits.TON = 1;  //turn on the timer3 to scan Keypad
            DELAY_MS(300);
            if (u8_newKey == '#') {
                e_state = STATE_SETUP_CW3;
            }
            else 
                e_state = STATE_SETUP_CW1_LCD_ON;
            break;
            
        case STATE_SETUP_CW3 :
            T3CONbits.TON = 0;  //turn off the timer3 to disable Keypad
            u8_eval = evaluateCW(u16_max);
            if(u8_eval == 1) {
                u16_cw = u16_max;
                min_pw = CENTER;
                max_pw = CENTER + 1;
                setServoRange();
                e_state = STATE_SETUP;
            } else {
                writeLCD(0x01,0,0,1); //clear screen and go to 1st line
                outStringLCD("Invalid value");
                writeLCD(0xC0,0,0,1);//goto second line, digit 7
                outStringLCD("Try again");
                DELAY_MS(1500);
                e_state = STATE_SETUP_CW;
            }
            break;
            
        case STATE_SETUP_CCW_CONF :
            T3CONbits.TON = 0;  //turn off the timer3 to disable Keypad
            min_pw = MIN_PW;
            max_pw = CENTER + 1;
            setServoRange();
            e_state = STATE_SETUP_CCW;
            break;
            
        case STATE_SETUP_CCW :
            writeLCD(0x01,0,0,1); //clear screen and go to 1st line
            outStringLCD("Set up CCW value");
            writeLCD(0xC0,0,0,1);//go to 2nd line
            outStringLCD("0deg to 90deg");
            DELAY_MS(1500);
            e_state = STATE_SETUP_CCW1;
            break;
        
        case STATE_SETUP_CCW1_LCD_ON :
            T3CONbits.TON = 0;  //turn off the timer3 to disable Keypad
            e_state = STATE_SETUP_CCW1;
            break;
        
        case STATE_SETUP_CCW1 :
            u16_min = ticksToUs(OC1RS, getTimerPrescale(T2CONbits));
            u16_ccw = (u16_min - MIN_PW) / 10;
            writeLCD(0x01,0,0,1); //clear screen and go to 1st line
            sprintf(lcdoutput, "%d deg", u16_ccw);
            outStringLCD(lcdoutput);
            writeLCD(0x88,0,0,1); //move cursor to the right
            sprintf(lcdoutput, "(%dus)", u16_min);
            outStringLCD(lcdoutput);
            writeLCD(0xC7,0,0,1);//go to 2nd line and 8th location
            outStringLCD("[#:ENTER]");
            e_state = STATE_SETUP_CCW2;
            break;
            
        case STATE_SETUP_CCW2 :
            E_LOW();            //Disable LCD
            DRIVE_ROW_LOW();    //drive rows of Keypad low 
            T3CONbits.TON = 1;  //turn on the timer3 to scan Keypad
            DELAY_MS(300);
            if (u8_newKey == '#')
                e_state = STATE_SETUP_CCW3;
            else 
                e_state = STATE_SETUP_CCW1_LCD_ON;
            break;
            
        case STATE_SETUP_CCW3 :
            T3CONbits.TON = 0;  //turn off the timer3 to disable Keypad
            u8_eval = evaluateCCW(u16_min);
            if(u8_eval == 1) {
                u16_ccw = u16_min;
                min_pw = CENTER;
                max_pw = CENTER + 1;
                setServoRange();
                BlinkLED(2);
                e_state = STATE_OPTIONS;
            } else {
                writeLCD(0x01,0,0,1); //clear screen and go to 1st line
                outStringLCD("Invalid value");
                writeLCD(0xC0,0,0,1);//goto second line, digit 7
                outStringLCD("Try again");
                DELAY_MS(1500);
                e_state = STATE_SETUP_CCW;
            }
            break;
            
        case STATE_RUN :
            T3CONbits.TON = 0;  //turn off the timer3 to disable Keypad
            if (evaluateCCW(u16_ccw) == 1 && evaluateCW(u16_cw) == 1) {
                e_state = STATE_RUN_CONF;
            } else if (evaluateCW(u16_cw) == 0) {
                writeLCD(0x01,0,0,1); //clear screen and go to 1st line
                outStringLCD("Invalid CW");
                writeLCD(0xC0,0,0,1);//goto second line
                outStringLCD("Try again");
                DELAY_MS(2000);
                e_state = STATE_OPTIONS;
            } else {
                writeLCD(0x01,0,0,1); //clear screen and go to 1st line
                outStringLCD("invalid CCW");
                writeLCD(0xC0,0,0,1);//goto second line
                outStringLCD("Try again");
                DELAY_MS(2000);
                e_state = STATE_OPTIONS;
            }
            break;
            
        case STATE_RUN_CONF :
            writeLCD(0x01,0,0,1); //clear screen and go to 1st line
            outStringLCD("Configuration...");
            min_pw = u16_ccw;   // CCW range
            max_pw = u16_cw;    // CW range
            setServoRange();
            DELAY_MS(300);
            e_state = STATE_RUN_STAT1;
            break;
        
        case STATE_RUN_STAT1 :
            T3CONbits.TON = 0;  //turn off the timer3 to disable Keypad
            u32_pw = ticksToUs(OC1RS, getTimerPrescale(T2CONbits));
            writeLCD(0x01,0,0,1); //clear screen and go to 1st line
            sprintf(lcdoutput, "%ld deg", (u32_pw-MIN_PW)/10);
            outStringLCD(lcdoutput);
            writeLCD(0x88,0,0,1); //move cursor to the right
            sprintf(lcdoutput, "(%ldus)", u32_pw);
            outStringLCD(lcdoutput);
            e_state = STATE_RUN_CALC;
            break;
        
        case STATE_RUN_CALC :
            f_cell = u32_pw;
            f_cell = ((f_cell-MIN_PW)/RANGE) * 12;
            u8_cell = f_cell;
            f_bar = f_cell - u8_cell;
            u8_bar = f_bar * 5;
            writeLCD(0xC0,0,0,1); //goto second line
            for(i = 0; i < u8_cell; i++)
                writeLCD(4,1,1,1);
            if(u8_cell == 11) {
                writeLCD(u8_bar,1,1,1);
            } else if(u8_bar > 0) {
                writeLCD((u8_bar-1),1,1,1);
            } else if(u8_cell == 0 && u8_bar == 0) {
                e_state = STATE_RUN_STAT2;
            }
            e_state = STATE_RUN_STAT2;
            break;
        
        case STATE_RUN_STAT2 :
            writeLCD(0xCC,0,0,1); //go to 2nd line and 13th location
            f_cell = u32_pw;
            sprintf(lcdoutput, "%.0f", ((f_cell-MIN_PW)/RANGE)*100);
            outStringLCD(lcdoutput);
            outStringLCD("%");
            e_state = STATE_RUN_KEY_ON;
            break;
        
        case STATE_RUN_KEY_ON :
            E_LOW();            //Disable LCD
            DRIVE_ROW_LOW();    //drive rows of Keypad low 
            T3CONbits.TON = 1;  //turn on the timer3 to scan Keypad
            DELAY_MS(500);
            if(u8_newKey == '#') {
                T2CONbits.TON = 0;      //turn off Timer2
                e_state = STATE_OPTIONS_LCD_ON;
            }
            else
                e_state = STATE_RUN_STAT1;
            break;
        
        case STATE_DOWNLOAD :
            T3CONbits.TON = 0;  //turn off the timer3 to disable Keypad
            writeLCD(0x01,0,0,1); //clear screen and go to 1st line
            outStringLCD("Sending messages");
            while (!RCFGCALbits.RTCSYNC) doHeartbeat();
            readRTCC();
            printRTCC();
            DELAY_MS(30);
            printf("PWM Pulse Width: %ld us\n", u32_pw);
            DELAY_MS(1000); //delay so that we do not flood the UART.
            e_state = STATE_DOWNLOAD1;
            break;
        
        case STATE_DOWNLOAD1 :
            min_pw = CENTER;
            max_pw = CENTER + 1;
            setServoRange();
            writeLCD(0x01,0,0,1); //clear screen and go to 1st line
            outStringLCD("Done");
            BlinkLED(3);
            e_state = STATE_OPTIONS;
            break;
                
        default :
            e_state = STATE_OPTIONS;
    } // end of switch (e_state)
    u8_newKey = 0;
} // end of update_state()


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main (void) {
  __builtin_write_OSCCONL(OSCCON | 0x02);    //  OSCCON.SOSCEN=1;
  configBasic(HELLO_MSG);
  /* Configure the LED */
  CONFIG_LED1();  
  LED1 = 0;
  /* Configure the LCD control lines */
  configControlLCD();  
  initLCD();
  /* Configure Timer3 for Keypad */
  configTimer3();
  /* Configure Timer2 and OC1 */
  configTimer2();
  configOutputCompare1();
  /* Configure ADC*/
  CONFIG_RA1_AS_ANALOG();
  configADC1_ManualCH0(RA1_AN, 31, 1);
  SET_SAMP_BIT_ADC1();    //start sampling and conversion
  /* Set initial PWM*/
  min_pw = MIN_PW;
  max_pw = MAX_PW + 1;
  setServoRange();
  /* Configure RTCC */
  outStringLCD("Please set date");
  getDateFromUser(); //get initial date/time
  setRTCC();         //set the date/time
  /* Input bar characters */
  LCD_build(0,bar1);
  LCD_build(1,bar2);
  LCD_build(2,bar3);
  LCD_build(3,bar4);
  LCD_build(4,bar5);
  
  /* Main function */
    while (1) {
        update_state();
        doHeartbeat();  //ensure that we are alive
    }
}
