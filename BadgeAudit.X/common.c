#include "common.h"

void led_setup(void)
{
    TRISA &= 0b11000000;    //Green LEDs pin: set to output
    PORTA &= 0b11000000;    //Green LEDs OFF

    TRISB &= 0b11011111;    //IR LED pin
    PORTB &= 0b11011111;    //IR LED OFF

    TRISC &= 0b11111000;    //Last two Green LED and Red Status
    PORTC &= 0b11111000;    //C port LEDs OFF

    LATCbits.LATC2 = 1;     //turn on red status LED
    TRISBbits.RB2 = 1;      //set RB2 to input
    TRISBbits.RB3 = 0;      //speaker output
}

void serial_setup(void)
{
    stdout = _H_USART;      //setup printf to use RS232

    BAUDCON = 0x00;         //Baud rate control register
    TXSTAbits.TX9 = 0;      //8 bit transmission
    TXSTAbits.TXEN = 1;     //enable transmit
    TXSTAbits.SYNC = 0;     // Asychronous Mode
    TXSTAbits.BRGH = 1;     //high speed mode

    RCSTAbits.SPEN = 1;     //USB circuitry in power conserve mode
    RCSTAbits.RX9 = 0;      //8 bit
    RCSTAbits.CREN = 1;     //enable receiver

    SPBRG = 25;             //set to 25 to get ~115200 baud

    TRISCbits.RC6 = 0;      //TX pin needs to be output
    TRISCbits.RC7 = 1;      //RX pin needs to be input
}

void interrupt_setup(void)
{
    //reset interrupt control regs
    INTCON = 0;
    INTCON2 = 0;
    INTCON3 = 0;

    //Port B interrupt
    INTCON3bits.INT2IP = 0;         //INT2 low priority
    INTCON3bits.INT2IF = 0;         //Clear INT2 flag
    INTCON2bits.RBPU = 1;           //pull up enable
    INTCON2bits.INTEDG2 = 0;        //rising/falling (1/0)
    INTCON3bits.INT2IE = 1;         //Enable INT2

    //---------------------
    //Timer Interrupt Setup
    //---------------------
    INTCONbits.TMR0IF = 0;          //Clear timer 0 flag
    PIR1bits.TMR1IF = 0;            //Clear time 1 flag

    RCONbits.IPEN = 1;              //Interrupt priority enable (0 = off)

    //Timer 0 setup
    INTCONbits.TMR0IE = 1;          //Enable Timer 0
    T0CONbits.T08BIT = 0;           //timer 0 set to 16 bit (1 = 8 bit)
    T0CONbits.T0PS = 0b111;          //Timer 0 Prescalar select (0b111 = 1:256)
    T0CONbits.T0CS = 0;             //Timer 0 source select (0 = inter inst. clk
    T0CONbits.PSA = 1;              //Timer 0 turn scalar on
    INTCON2bits.TMR0IP = 1;         //Timer 0 interrupt priority (1 = high)

    //Timer 1 setup
    T1CONbits.TMR1CS = 0;           //Timer 1, external or internal osc
    T1CONbits.T1CKPS = 0b01;        //Timer 1 Prescalar Select (0b01 = 1:2)
    IPR1bits.TMR1IP = 1;            //Timer 1 overflow priority
    PIE1bits.TMR1IE = 1;            //Enable timer 1 overflow interrupt

    //Timer 2 Setup
//    PR2 = 0b10111011;               //set timer 2 period
//    T2CON = 0;                      //initialize to zero
//    T2CONbits.TMR2ON = 1;           //Turn timer 2 on
//    T2CONbits.T2CKPS = 0b11;        //prescalar 16
//
//    ADCON1 = 0xf;                   //make AN* ports digital I/O
//
//    //set 50% duty cycle
//    CCPR1L = 0b01011101;
//    CCP1CON = 0b00111100;

    INTCONbits.PEIE = 1;            //Peripheral interrupt enable
    INTCONbits.GIE = 1;             //enable global interrupt

    TMR0L = 0;                      //Timer 0 counts up from 208
    TMR0H = TONE_LOW_BYTE;

    TMR1L = 0;                      //Timer 1 counts up from 0
    TMR1H = 0;

    T0CONbits.TMR0ON = 0;           //turn timer 0 on/off (1/0)
    T1CONbits.TMR1ON = 0;           //turn timer 1 on/off (1/0)
}

void i2c_setup(void)
{
    //-------------------
    //Setup I2C
    //-------------------

    SSPADD = 0x78;                  //120
    OpenI2C(MASTER, SLEW_OFF);      //Init I2C module

    printf("\n\n\rConfiguring I2C...");

    StartI2C();
        WriteI2C(0x98);
        WriteI2C(0x07);                //mode register
        WriteI2C(0x00);                //Standby mode
    StopI2C();
    IdleI2C();
    StartI2C();
        WriteI2C(0x98);
        WriteI2C(0x05);                //SPCNT register
        WriteI2C(0x00);                //No sleep count
    StopI2C();
    IdleI2C();
    StartI2C();
        WriteI2C(0x98);
        WriteI2C(0x06);                //interrupt register (INTSU)
        WriteI2C(0xE4);                //shake INT on 3-axis, and tap detect
    StopI2C();
    IdleI2C();
    StartI2C();
        WriteI2C(0x98);
        WriteI2C(0x09);                //PDET register
        WriteI2C(0x84);                //tap detection on  z-axis
    StopI2C();
    IdleI2C();
    StartI2C();
        WriteI2C(0x98);
        WriteI2C(0x08);                //SR register
        WriteI2C(0x00);                //Sample rate at 32/sec
    StopI2C();
    IdleI2C();
    StartI2C();
        WriteI2C(0x98);
        WriteI2C(0x0A);                //PD register
        WriteI2C(0x2F);                //test tap detection, debounce
    StopI2C();
    IdleI2C();
    StartI2C();
        WriteI2C(0x98);            
        WriteI2C(0x07);             //select the mode register
        WriteI2C(0x41);             //active mode, int push pull
    StopI2C();

    printf("Finished!\r\n");
}

void setup(void)
{
    led_setup();

    serial_setup();
    interrupt_setup();
    i2c_setup();

    INTCON3bits.INT2IF = 0;

    //Print a welcome message in case someone plugs it up
    printf("Welcome to the RVAsec Badge!\n\r\n\r");
}