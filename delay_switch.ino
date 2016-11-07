  //Includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <EEPROM.h>
#include <avr/wdt.h>

#define BUZZERPIN           0 // Connect to piezo buzzer
#define SWPIN               1 // Connect to the switch 
#define HARTBEATPIN         3 // This pin used to monitor timer
#define FANPIN              4 // Connect to relay circuit 

// State machine state
#define ST_INIT             0
#define ST_FAN_ON           1
#define ST_WAIT_OFF         2
#define ST_FAN_OFF          3
#define ST_DETENTION        4
#define ST_PRE_SETTING      5
#define ST_SETTING_1        6
#define ST_SETTING_2        7
#define ST_SETTING_OK_1     8
#define ST_SETTING_OK_2     9

// Switch state
#define ON                  1
#define OFF                 0

// Buzzer
#define BZ_ENTER_SETTING    0
#define BZ_SETTING_OK       1
#define BZ_SETTING_FAILED   2
#define BZ_SHORT_BEEP       3

// Timer counter value
// increase 1 per 0.5 second
#define SHORT_TIMEOUT               1
#define MID_TIMEOUT                 3
#define LONG_TIMEOUT                8
#define DELAY_OFF_INCREMENT         360     // (3 * 60 * 2) increase 3 min / step
#define INST_OFF_SEC                20      // 10 sec
#define DEFAULT_DELAY_OFF_SEC       1440   // (12 * 60 *2) 12 min

byte                            SM_Current_St;      // State machine current state
byte                            SM_Last_St;         // State machine last state
unsigned int                    Delay_Off_sec;      // Delay timer to turn off fan
unsigned int                    tmp_Delay_Off_sec;  // temparory counter for setting stage
static volatile byte            SM_Next_St;         // State machine next state
static volatile unsigned int    Timer_Counter;      // Timer counter, the MAX value will be count to DELAY_OFF_SEC
static volatile byte            SW_St;              // Switch state ON/OFF
unsigned int                    Wdt_Counter;        // Watchdog counter

void fan(byte onoff) {
    digitalWrite(FANPIN, onoff);
}

void start_timer(void) {
    TCNT1 = 12; // TCNT1 preset
    // Set prescale to 2048(CS[13:10] = 0b1100) to start timer
    // 1 interrupt / 500ms
    TCCR1 |= (1<<CS13)|(1<<CS12);
    TCCR1 &= ~((1<<CS10)|(1<<CS11));
}

void stop_timer(void) {
    digitalWrite(HARTBEATPIN, LOW);
    // Set CS[13:10] = 0 to stop timer1
    TCCR1 &= ~((1<<CS13)|(1<<CS12)|(1<<CS11)|(1<<CS10));
    Timer_Counter = 0;
}

void reset_timer(void) {
    stop_timer();
    start_timer();
}

void beep(byte state) {
    stop_timer();
    switch (state) {
        case BZ_ENTER_SETTING:
            digitalWrite(BUZZERPIN, HIGH);
            delay(300);
            digitalWrite(BUZZERPIN, LOW);
            delay(100);
            digitalWrite(BUZZERPIN, HIGH);
            delay(100);
            digitalWrite(BUZZERPIN, LOW);
            break;
        case BZ_SETTING_OK:
            digitalWrite(BUZZERPIN, HIGH);
            delay(100);
            digitalWrite(BUZZERPIN, LOW);
            delay(100);
            digitalWrite(BUZZERPIN, HIGH);
            delay(300);
            digitalWrite(BUZZERPIN, LOW);
            break;
        case BZ_SETTING_FAILED:
            digitalWrite(BUZZERPIN, HIGH);
            delay(500);
            digitalWrite(BUZZERPIN, LOW);
            break;
        case BZ_SHORT_BEEP:
            digitalWrite(BUZZERPIN, HIGH);
            delay(100);
            digitalWrite(BUZZERPIN, LOW);
            break;
    }
    start_timer();
}

void write_eeprom(unsigned int val) {
    EEPROM.write(0, val >> 8);
    EEPROM.write(1, val & 0x00FF);
}

unsigned int read_eeprom(void) {
    unsigned int value;

    value = (EEPROM.read(0) << 8 | EEPROM.read(1));
    return value;
}

void setup() {
    unsigned int Delay_Off_sec_init;
    /* Initial variable */
    pinMode(SWPIN, INPUT);
    SW_St = digitalRead(SWPIN); // Read the current switch state
    SM_Current_St = (SW_St == HIGH)? ST_FAN_ON:ST_INIT;
    pinMode(HARTBEATPIN, OUTPUT);
    digitalWrite(HARTBEATPIN, LOW);
    pinMode(FANPIN, OUTPUT);
    digitalWrite(FANPIN, LOW);  // Turn off the fan
    pinMode(BUZZERPIN, OUTPUT);
    digitalWrite(BUZZERPIN, LOW);
    Delay_Off_sec_init = read_eeprom();
    Delay_Off_sec = (Delay_Off_sec_init == 0xFFFF)? DEFAULT_DELAY_OFF_SEC:Delay_Off_sec_init;
    Wdt_Counter = 0;
 
    cli();//disable interrupts during setup
    // Setup switch interrupt
    PCMSK |= (1 << PCINT1); // Enable PB1 change interrupt
    GIMSK |= (1 << PCIE);   // enable PCINT interrupt in the general interrupt mask
    DDRB &= ~(1 << DDB1);   // Set PB1 as input pin
    // Setup Timer1
 //   start_timer();
    TIMSK |= (1<<TOIE1);    // Timer Overflow Interrupt Enable

    // Setting Watchdog timer
    WDTCR |= (1 << WDCE) | (1 << WDE);
    // Watchdog timeout = 4s
    WDTCR = (0 << WDCE) | (1 << WDE) | (1 << WDP3) | (0 << WDP2) | (0 << WDP1) | (0 << WDP0);
    sei(); //last line of setup - enable interrupts after setup

}

void loop() {
    if(SM_Current_St != SM_Next_St) {
        SM_Last_St = SM_Current_St;
        SM_Current_St = SM_Next_St;
        
        switch(SM_Current_St) {
            case ST_INIT:
                stop_timer();
                break;
            case ST_DETENTION:
                start_timer();
                break;
            case ST_FAN_ON:
                reset_timer();
                fan(ON);
                break;
            case ST_PRE_SETTING:
                reset_timer();
                break;
            case ST_SETTING_1:
                if (SM_Last_St == ST_SETTING_2) {
                    tmp_Delay_Off_sec += DELAY_OFF_INCREMENT;
                    beep(BZ_SHORT_BEEP);
                } else
                    beep(BZ_ENTER_SETTING);
//                reset_timer();
                break;
            case ST_SETTING_2:
                reset_timer();
                break;
            case ST_SETTING_OK_2:
                if (tmp_Delay_Off_sec != 0) {
                    Delay_Off_sec = tmp_Delay_Off_sec;
                    write_eeprom(Delay_Off_sec);
                    tmp_Delay_Off_sec = 0;
                    beep(BZ_SETTING_OK);
                } else
                    beep(BZ_SETTING_FAILED);
//                reset_timer();
                SM_Next_St = ST_INIT;
                break;
            case ST_WAIT_OFF:
                reset_timer();
                break;
            case ST_SETTING_OK_1:
                if (tmp_Delay_Off_sec != 0) {
                    Delay_Off_sec = tmp_Delay_Off_sec;
                    write_eeprom(Delay_Off_sec);
                    tmp_Delay_Off_sec = 0;
                    beep(BZ_SETTING_OK);
                } else
                    beep(BZ_SETTING_FAILED);
//                reset_timer();
                SM_Next_St = ST_FAN_ON;
                break;
            case ST_FAN_OFF:
                fan(OFF);
                SM_Next_St = ST_INIT;
                break;
        }
    }
    Wdt_Counter++;
    if (Wdt_Counter > 0xfffe) {
        wdt_reset();
        Wdt_Counter = 0;
    }
}

// Switch ISR
ISR(PCINT0_vect)
{
    SW_St = (PINB >> PINB1) & 1; // PINB is the register to read the state of the pins
    if (SW_St == ON) {  // Switch: OFF -> ON
        switch (SM_Current_St) {
            case ST_INIT:
                SM_Next_St = ST_DETENTION;
                break;
            case ST_PRE_SETTING:
                if (Timer_Counter < SHORT_TIMEOUT)
                    SM_Next_St = ST_SETTING_1;
                break;
            case ST_SETTING_2:
                if (Timer_Counter < LONG_TIMEOUT)
                    SM_Next_St = ST_SETTING_1;
                break;
            case ST_WAIT_OFF:
                SM_Next_St = ST_FAN_ON;
                break;
        }
    } else {    // Switch: ON -> OFF
        switch (SM_Current_St) {
            case ST_DETENTION:
                if (Timer_Counter < SHORT_TIMEOUT)
                    SM_Next_St = ST_PRE_SETTING;
                break;
            case ST_SETTING_1:
                if (Timer_Counter < LONG_TIMEOUT)
                    SM_Next_St = ST_SETTING_2;
                break;
            case ST_FAN_ON:
                if (Timer_Counter < INST_OFF_SEC)
                    SM_Next_St = ST_FAN_OFF;
                else
                    SM_Next_St = ST_WAIT_OFF;
                break;
        }
    }
}

// Timer ISR
ISR(TIMER1_OVF_vect)
{
    TCNT1 = 12; // TCNT1 preset
    switch (SM_Current_St) {
        case ST_DETENTION:
            if (Timer_Counter >= SHORT_TIMEOUT)
                SM_Next_St = ST_FAN_ON;
            break;
        case ST_PRE_SETTING:
            if (Timer_Counter >= SHORT_TIMEOUT)
                SM_Next_St = ST_INIT;
            break;
        case ST_SETTING_1:
            if (Timer_Counter >= LONG_TIMEOUT)
                SM_Next_St = ST_SETTING_OK_1;
            break;
        case ST_SETTING_2:
            if (Timer_Counter >= LONG_TIMEOUT)
                SM_Next_St = ST_SETTING_OK_2;
            break;
        case ST_WAIT_OFF:
            if (Timer_Counter >= Delay_Off_sec)
                SM_Next_St = ST_FAN_OFF;
            break;
    }
    digitalWrite(HARTBEATPIN, !digitalRead(HARTBEATPIN));   // Toggle HARTBEATPIN
    Timer_Counter += 1;
}
