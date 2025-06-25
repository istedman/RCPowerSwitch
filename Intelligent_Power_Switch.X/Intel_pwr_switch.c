//***********************************************************************************
// Intelligent power switch code.
// Design to switch on a small power MOSFET on the supply side, based on either
// a R/C servo PWM input or a discrete input.
// Switch on time is user programmable as is the option to use either the PWM 
// input or the discrete input.

// Device: PIC12F1822
// Compiler: Microchip XC8 v1.10
// IDE: MPLAB X v5+
// Created: 27th April 2020
//
// In this simple example we have a 5V pulse with a width that ranges from 1ms to 2ms
// coming into the Timer 1 Gate Pin (RA4).  This example uses the TMR1 Gate
// feature set up in 'Single Pulse Mode' to generate an interrupt when a pulse
// has been received.  Based on the width of the pulse, an LED (pin RA2) will
// be turned on or off

// **********************************************************************************
// set Config bits
/*
#pragma config FOSC=INTOSC, WDTE=OFF, MCLRE=OFF, CP=OFF, CPD=OFF,BOREN=OFF
#pragma config CLKOUTEN=OFF, IESO=OFF, FCMEN=OFF,FOSC = INTOSC, FCMEN=OFF
#pragma config WRT=OFF,PLLEN=OFF,STVREN=ON,BORV=LO,LVP=OFF,PWRTE=ON, DEBUG=OFF
*/
        
//Debug config

#pragma config FOSC=INTOSC, WDTE=OFF, MCLRE=ON, CP=OFF, CPD=OFF,BOREN=OFF
#pragma config CLKOUTEN=OFF, IESO=OFF, FCMEN=OFF,FOSC = INTOSC, FCMEN=OFF
#pragma config WRT=OFF,PLLEN=OFF,STVREN=ON,BORV=LO,LVP=OFF

#include <xc.h> // include standard header file




// Definitions
#define _XTAL_FREQ  16000000        // this is used by the __delay_ms(xx) and __delay_us(xx) functions
#define PWM_In      PORTAbits.RA4   // RA4 is PWM input signal (this is T1G input)
#define LED1        LATAbits.LATA0  // RA0 is LED for debugging purposes
#define LED2        LATAbits.LATA1  // RA1 is second debug LED
#define PWRSW       LATAbits.LATA2  // RA2 is MOSFET drive output
#define SETUPSW     PORTAbits.RA5   // Setup switch
#define EXTSW       PORTAbits.RA3   // External switch or logic input

#define LEDON  0
#define LEDOFF 1
#define POWERON 1
#define POWEROFF 0
#define DEBOUNCETIME 150             // Switch debounce time in ms
#define SWITCHCLOSED 0
#define SWITCHOPEN 1
#define GREATERTHAN 0x33
#define LESSTHAN    0x22
#define MAXONPULSE  40000           //2.5ms @ 62.5ns tick time
#define MINONPULSE  8000            //0.5ms @ 62.5ns tick time
/* Help adjust pulse value by adjusting by a set number of microseconds from the actual numbers
 * to allow for some jitter in the received pulse.
*/
#define TimerAdjustment 144     // 9000ns/62.5 = 144 

#define DEBUG 2

// Global Variables   
//__EEPROM_DATA(0x51,0x40,0x33,00,00,00,00,00);  // Default pulse limit of 20800 = 0x5140, 0x33 means look for greater than pulse value
__EEPROM_DATA(0x7A,0x93,0x33,02,53,55,00,00);  // Default pulse limit of 20800 = 0x5140, 0x33 means look for greater than pulse value
// 16MHz = 62.5ns, 20800x62.5 = 1.3ms, the default time.
unsigned int PulseLimit=0;
unsigned char Plimitlo, Plimithi;
unsigned char GreatOrLessThan;
volatile unsigned int PulseValue;   //this will contain our counter value
struct
{
  unsigned PulseFound :1;  // flag used to indicate that pulse value has been grabbed
}flags;


//*************************************************************************************

void __interrupt() Timer1_Gate_ISR(void)
{
    PulseValue = TMR1;  // read timer value
    flags.PulseFound=1; // set flag for main routine to use
    TMR1=0;             // clear timer value
    PIR1bits.TMR1GIF=0; // clear the the interrupt flag
    T1GCONbits.T1GGO=1; // set the T1GGO/Done bit so it starts looking for a pulse again

}

void PicInit12F1822()
{
       // set up oscillator control register
    OSCCONbits.SPLLEN=0;    // PLL is disabled
    OSCCONbits.IRCF=0x0F;   //set OSCCON IRCF bits to select OSC frequency=16Mhz
    OSCCONbits.SCS=0x02;    //set the SCS bits to select internal oscillator block
    // OSCON should be 0x7Ah now.

    // Set up I/O pins
    ANSELAbits.ANSELA=0;    // set all analog pins to digital I/O
    ADCON0bits.ADON=0;      // turn ADC off
    DACCON0bits.DACEN=0;    // turn DAC off
    
    //TRISA=0x10; // PORT A Assignments
    TRISAbits.TRISA0 = 0;	// RA0 = SET_LED1, 0 = on, 1 = off.
    TRISAbits.TRISA1 = 0;	// RA1 = SET_LED2, 0 = on, 1 = off.
    TRISAbits.TRISA2 = 0;	// RA2 = MOSFET gate drive, 1 = on, 0 = off.
    TRISAbits.TRISA3 = 1;	// RA3 = Discrete input, user programmable
    TRISAbits.TRISA4 = 1;	// RA4 = PWM pulse input to gate
    TRISAbits.TRISA5 = 1;	// RA5 = Setup switch, internal pull-up enabled.
    // Weak pull-up enables
    WPUAbits.WPUA5 = 1;     // Enable weak pull-up on RA5 (switch input)
    WPUAbits.WPUA3 = 1;     // Enable pull-up on mCLR.
    WPUAbits.WPUA4 = 1;     // Enable pull-up on PWM input, to allow for no-connect case.
    OPTION_REGbits.nWPUEN = 0; // Enable global weak pullups in OPTION_REG
    
    // Set up the Timer1 Control Register (T1CON)
    T1CONbits.TMR1CS=0x01;  // Timer 1 source is system clock (FOSC)
    T1CONbits.T1CKPS0=0;    // these 2 bits set prescalar 1:1
    T1CONbits.T1CKPS1=0;
    T1CONbits.T1OSCEN=0;    // dedicated Timer1 oscillator circuit disabled
    T1CONbits.nT1SYNC=1;    // Do not synchronize external clock input
    T1CONbits.TMR1ON=1;     // Timer 1 on

    // Now set up the Timer1 Gate Control Register (T1GCON)
    T1GCONbits.TMR1GE=1;    // Timer1 gate enable is on
    T1GCONbits.T1GPOL=1;    // gate is active high (TMR1 counts when gate is high)
    T1GCONbits.T1GTM=0;     // gate toggle mode is disabled
    T1GCONbits.T1GSPM=1;    // single pulse mode is enabled and controlling the gate
    T1GCONbits.T1GSS=0x00;  // Timer1 Gate Source is the Timer1 Gate Pin
    // T1GCON should be 1101xx00b at this point


    TMR1=0;                 // set timer1 back to zero
    PIR1bits.TMR1GIF=0;     // make sure the peripheral interrupt flag is clear
    INTCONbits.PEIE=1;      // Enable peripheral interrupt
    INTCONbits.GIE=1;       // enable global interrupt
    PIE1bits.TMR1GIE=1;     // enable Timer 1 Gate interrupt
    T1GCONbits.T1GGO=1;     // set the T1GGO/Done bit so it starts looking for a pulse

}
/* Function to do the setup for a simple on/off switch.
 * Timers have been set by PicInit12F1822, so use them to read the pulses
 * It can work with greater than a pulse limit or less than a pulse limit. This si stored in EEPROM
 *
 */ 
int DoOnOffSetup()
{
    unsigned int Pulse1, Pulse2;
    unsigned char Byte1, Byte2;
    unsigned char Dbounce1=0, Dbounce2=0;
    LED2=LEDON;     // Indicate we are in setup mode.
    LED1=LEDON;
    //Wait for switch release
    do
    {
        __delay_ms(150);
    }while(SETUPSW==SWITCHCLOSED);
    
    __delay_ms(1000);
    
    // Now we need the off position, wait for user to press button
    do
    {
        Dbounce1=SETUPSW;
        LED1= !LED1; //Toggle LED1
        __delay_ms(DEBOUNCETIME);
        Dbounce2=SETUPSW;        
    } while(Dbounce1==SWITCHOPEN && Dbounce2==SWITCHOPEN );
    
    flags.PulseFound=0;     // Clear interrupt?
    do
    {
        __delay_ms(1);
    } while(flags.PulseFound!=1);
    
    //Button has been pressed and debounced, read initial/off value        
     di();                       // disable all interrupts to read the counter value
        Pulse1 = PulseValue;   // get the timer value
     ei();                       // re-enable interrupts again
     flags.PulseFound=0;
    
    LED1=LEDON;
    __delay_ms(1000);
    LED1=LEDOFF;
 
    //Now we need the full extent/on value, let user release switch
    do
    {
        Dbounce1=SETUPSW;
        LED1=!LED1;
        __delay_ms(DEBOUNCETIME);
        Dbounce2=SETUPSW;        
    } while(Dbounce1==SWITCHCLOSED && Dbounce2==SWITCHCLOSED );      
    
    LED1=LEDON;
    //Now move to final value and close switch.
    // Now we need the off position, wait for user to press button
    
     do
    {
        Dbounce1=SETUPSW;
        LED1=!LED1;
        __delay_ms(DEBOUNCETIME);
        Dbounce2=SETUPSW;        
    } while(Dbounce1==SWITCHOPEN && Dbounce2==SWITCHOPEN );  
    
    flags.PulseFound=0;     // Clear interrupt?
    do
    {
        __delay_ms(1);
    } while(flags.PulseFound!=1);
    
     di();                       // disable all interrupts to read the counter value
        Pulse2 = PulseValue;   // get the timer value
     ei();                       // re-enable interrupts again
     flags.PulseFound=0;
     LED1=LEDOFF;
    __delay_ms(1000);
    LED2=LEDON;
    /* We should have two pulse limits in pulse1 & pulse2
     * If we subtract pulse2 from pulse 1 and it is positive, we are looking for greater than pulse2-TimerAdjustment
     * so we store pulse2 and GREATERTHAN to EEPROM
     * otherwise if pulse2-pulse1 is negative, we are looking for less than pulse 2 so we store pulse2+TimerAdjustmen
     * and LESSTHAN to EEPROM
     */ 
 
    if(Pulse2>Pulse1)
    {
        Pulse2=Pulse2-TimerAdjustment;
        Byte1=(unsigned char)(Pulse2>>8);
        Byte2=(unsigned char)(Pulse2&0xFF);
        EEPROM_WRITE(0,Byte1);
        EEPROM_WRITE(1,Byte2);
        EEPROM_WRITE(2,GREATERTHAN);
        
        #ifdef DEBUG
        Byte1=(unsigned char)(Pulse1>>8);
        Byte2=(unsigned char)(Pulse1&0xFF);
        EEPROM_WRITE(3,Byte1);
        EEPROM_WRITE(4,Byte2);
        #endif
    }
    else if (Pulse1>Pulse2)
    {
        Pulse2=Pulse2+TimerAdjustment;
        Byte1=(unsigned char)(Pulse2>>8);
        Byte2=(unsigned char)(Pulse2&0xFF);
        EEPROM_WRITE(0,Byte1);
        EEPROM_WRITE(1,Byte2);
        EEPROM_WRITE(2,LESSTHAN);
        #ifdef DEBUG
        Byte1=(unsigned char)(Pulse1>>8);
        Byte2=(unsigned char)(Pulse1&0xFF);
        EEPROM_WRITE(3,Byte1);
        EEPROM_WRITE(4,Byte2);
        #endif
    }
    //Now flash LEDs to indicate it is done
    LED1=LEDOFF;
    LED2=LEDON;
    __delay_ms(250);
    LED1=LEDON;
    LED2=LEDOFF;  
    __delay_ms(250);
    LED1=LEDOFF;
    LED2=LEDON;   
    __delay_ms(250);
    LED1=LEDON;
    LED2=LEDOFF;    
    __delay_ms(250);
    LED1=LEDOFF;
    LED2=LEDOFF;
    // Back to main routine
    return(23);
}

//*************************************************************************************
int main ( )
{
    unsigned int Timer1Count;   // this will contain the Timer1 count value
    unsigned int KeyPressTime=0;
    PicInit12F1822();

   
    flags.PulseFound=0;     // initialize variables
    PulseValue=0;
  
    //See if we are in setup mode at power-on
    if (SETUPSW==SWITCHCLOSED)
    {
        __delay_ms(DEBOUNCETIME);   // Wait for debounce of switch
        __delay_ms(250);
        if(SETUPSW==SWITCHCLOSED)
        {
            DoOnOffSetup();              // Switch valid/debounced, do the setup
        }
    }       // Else run normal loop

    // Recall EEPROM settings, either recently updated from above or the last stored.
    
    Plimithi=EEPROM_READ(0);
    Plimitlo=EEPROM_READ(1);
    PulseLimit= (unsigned int) Plimitlo | (unsigned int) Plimithi<<8; // Recall save Pulse limit
    GreatOrLessThan=EEPROM_READ(2); // See if we look for greater than or less than
    LED2=LEDOFF;
    LED1=LEDOFF;
    
    do{ // main loop
        if (flags.PulseFound==1)
        {
           // here is where you would read the PWM value and do something with it.

            di();                       // disable all interrupts to read the counter value
            Timer1Count = PulseValue;   // get the timer value
            ei();                       // re-enable interrupts again
            /*
             Now look for a pulse greater than the default but bound it to cater for no signal
             */
            if ((GreatOrLessThan==GREATERTHAN && Timer1Count > PulseLimit) &&
               (GreatOrLessThan==GREATERTHAN && Timer1Count < MAXONPULSE))    
            //if (GreatOrLessThan==GREATERTHAN && Timer1Count > PulseLimit && Timer1Count < MAXONPULSE )
            {
               LED1=LEDON;                       // on the width of the pulse
               LED2=LEDOFF;
               PWRSW=POWERON;
            }     
            /*
             Now look for a pulse less than the default but bound it to cater for no signal
             */
            else if ((GreatOrLessThan==LESSTHAN && Timer1Count < PulseLimit)&&
                     (GreatOrLessThan==LESSTHAN && Timer1Count > MINONPULSE))
            //else if ((GreatOrLessThan==LESSTHAN && Timer1Count < PulseLimit && Timer1Count > MINONPULSE))
            {
                LED1=LEDON; 
                LED2=LEDOFF;
                PWRSW=POWERON;
            }
            else
            {
                LED1=LEDOFF;  
                LED2=LEDOFF;
                PWRSW=POWEROFF;
            }
          
            flags.PulseFound=0;    // clear flag
          
        }
        
        else    // No signal found
        {
            NOP(); // or do other stuff if pulse not found
            LED1=LEDOFF;
            LED2=LEDON;     // Indicate no signal seen with a red LED.
            PWRSW=POWEROFF;
           
        }
        
        __delay_ms(25); 

    } while (1);


}
