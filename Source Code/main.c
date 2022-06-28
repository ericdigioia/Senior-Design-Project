/* 
 * File:   main.c
 * Author: Louis Eric DiGioia
 *
 * Created on February 27, 2021, 6:33 PM
 */

// <PIC24FJ64GA002>

// MCU internal clock frequency: 32 MHz after PLL
// Controller read frequency: 60 or 30 Hz

#pragma config FNOSC = FRCPLL // fast osc with PLL
#pragma config FWDTEN = OFF   // disable watchdog timer
#pragma config IOL1WAY = OFF  // IOLOCK may be unlocked via special sequence
#pragma config JTAGEN = OFF   // JTAG port disabled
#pragma config IESO = OFF     // IESO mode (Two-Speed Start-up) disabled
#pragma config OSCIOFNC = ON  // OSC2/CLKO/RC15 functions as port I/O (RC15)

#ifndef FCY
#define FCY 32000000UL  // PIC Fosc 32 MHz (for built-in delays)
#endif

// Some code configs
#define USE_GYRO                1   // Use I2C gyro, else enable OBC sel + start
                                    //  These 2 buttons disabled when using gyro

#define FIX_ORIENTATION         1   // Fix orientation when gyro is mounted upside down
                                    //  By chance, default is correct orientation
                                    //  so this bit does not affect functionality

#define CONTROLLER_READ_60HZ    0   // Read controller at 60 Hz, else 30 Hz read
                                    //  60 Hz not necessary for this application

#define QUICKBOOT               0   // Do not fill screen with white on boot
                                    //  Mostly to save some time debugging

// Some SPI defines for code readability
#define SPI1_SCL_LOW()   LATBbits.LATB8 = 0;    // SCL = RB8
#define SPI1_SCL_HIGH()  LATBbits.LATB8 = 1;
#define SPI1_SDA_LOW()   LATBbits.LATB9 = 0;    // SDA = RB9
#define SPI1_SDA_HIGH()  LATBbits.LATB9 = 1;
#define SPI1_SDA         PORTBbits.RB9
#define MPU6050_ADDR     0x68                   // MPU6050 slave address
#define I2C_READ         1
#define I2C_WRITE        0

// C includes                                   // Origins
#include <stdio.h>                              // Standard
#include <stdlib.h>                             // Standard
#include <string.h>                             // Standard
#include <stdbool.h>                            // Standard
#include <libpic30.h>                           // MicroChip
#include <p24FJ64GA002.h>                       // MicroChip
#include <xc.h>                                 // MicroChip
#include "i2c.h"                                // MicroChip
#include "ILI9340.h"                            // Custom/Adapted GFX library

/*
// Flash program writing stuff
_prog_addressT big_addr;
big_addr = __builtin_tbladdress(&_PROGRAM_END);
*/

// State Defines for System Control
#define STATE_SYSTEM_BOOT   0
#define STATE_SDCARD_CHECK  1
#define STATE_BIOS_MAINMENU 2
#define STATE_SYS_TEST_MENU 3
#define STATE_SYS_LANG_MENU 4
#define STATE_BOOT_SDCARD   5
#define STATE_SYS_CONT_TEST 6
#define STATE_SYS_SCRN_TEST 7
#define STATE_SYS_SND_TEST  8
#define STATE_SYS_GYRO_TEST 9
#define STATE_SYS_ERROR     10
#define STATE_SUPER_MARIO   11
// First state
volatile unsigned short current_state = STATE_SYSTEM_BOOT;
volatile bool newState = 1; // set when entering a new state

// System Language Control
#define LANG_EN 0
#define LANG_ES 1
#define LANG_PT 2
#define LANG_FR 3
#define LANG_JP 4
// Default language
volatile short sys_lang = LANG_EN;

// screen defines for misc logic
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

// Sound engine defines
#define DAC_CTRL_CODE 0b10010000 // control code 1001 to
                                 // load DAC A and update outputs

// interrupt declarations
void __attribute__((interrupt(no_auto_psv))) _T1Interrupt(void);
void __attribute__((interrupt(no_auto_psv))) _T2Interrupt(void);
void __attribute__((interrupt(no_auto_psv))) _T3Interrupt(void);

// controller variables (updated at 60 or 30 Hz)
volatile bool isPadConnected;
volatile uint8_t buttons;
volatile bool button_A;
volatile bool button_B;
volatile bool button_Select;
volatile bool button_Start;
volatile bool button_Up;
volatile bool button_Down;
volatile bool button_Left;
volatile bool button_Right;

// OBC variables
#define DEADZONEHI 600          // analog stick dead zone
#define DEADZONELO 400          // analog stick dead zone
volatile uint16_t thumb_x = 0;  // ADC reading for x-axis
volatile uint16_t thumb_y = 0;  // ADC reading for y-axis
volatile uint16_t ext_pad_check_cooldown_timer = 0; // cooldown timer to check if ext gamepad is reconnected
#define PADCHECKCOOLDOWN 60     // how long to wait until ext gamepad connection is checked again (1 sec at 60Hz read, 2 sec at 30Hz read)
bool read_y_axis = false;       // control variable for alternating reading x and y axis pots in analog stick

// OBC - Gyro variables
volatile int Ax, Ay, Az, Gx, Gy, Gz, T;

// MPU6050 Register Map
#define XG_OFFS_TC         0x00
#define YG_OFFS_TC         0x01
#define ZG_OFFS_TC         0x02
#define X_FINE_GAIN        0x03
#define Y_FINE_GAIN        0x04
#define Z_FINE_GAIN        0x05
#define XA_OFFS_H          0x06
#define XA_OFFS_L_TC       0x07
#define YA_OFFS_H          0x08
#define YA_OFFS_L_TC       0x09
#define ZA_OFFS_H          0x0A
#define ZA_OFFS_L_TC       0x0B
#define XG_OFFS_USRH       0x13
#define XG_OFFS_USRL       0x14
#define YG_OFFS_USRH       0x15
#define YG_OFFS_USRL       0x16
#define ZG_OFFS_USRH       0x17
#define ZG_OFFS_USRL       0x18
#define SMPLRT_DIV         0x19
#define CONFIG             0x1A
#define GYRO_CONFIG        0x1B
#define ACCEL_CONFIG       0x1C
#define FF_THR             0x1D
#define FF_DUR             0x1E
#define MOT_THR            0x1F
#define MOT_DUR            0x20
#define ZRMOT_THR          0x21
#define ZRMOT_DUR          0x22
#define FIFO_EN            0x23
#define I2C_MST_CTRL       0x24
#define I2C_SLV0_ADDR      0x25
#define I2C_SLV0_REG       0x26
#define I2C_SLV0_CTRL      0x27
#define I2C_SLV1_ADDR      0x28
#define I2C_SLV1_REG       0x29
#define I2C_SLV1_CTRL      0x2A
#define I2C_SLV2_ADDR      0x2B
#define I2C_SLV2_REG       0x2C
#define I2C_SLV2_CTRL      0x2D
#define I2C_SLV3_ADDR      0x2E
#define I2C_SLV3_REG       0x2F
#define I2C_SLV3_CTRL      0x30
#define I2C_SLV4_ADDR      0x31
#define I2C_SLV4_REG       0x32
#define I2C_SLV4_DO        0x33
#define I2C_SLV4_CTRL      0x34
#define I2C_SLV4_DI        0x35
#define I2C_MST_STATUS     0x36
#define INT_PIN_CFG        0x37
#define INT_ENABLE         0x38
#define DMP_INT_STATUS     0x39
#define INT_STATUS         0x3A
#define ACCEL_XOUT_H       0x3B
#define ACCEL_XOUT_L       0x3C
#define ACCEL_YOUT_H       0x3D
#define ACCEL_YOUT_L       0x3E
#define ACCEL_ZOUT_H       0x3F
#define ACCEL_ZOUT_L       0x40
#define TEMP_OUT_H         0x41
#define TEMP_OUT_L         0x42
#define GYRO_XOUT_H        0x43
#define GYRO_XOUT_L        0x44
#define GYRO_YOUT_H        0x45
#define GYRO_YOUT_L        0x46
#define GYRO_ZOUT_H        0x47
#define GYRO_ZOUT_L        0x48
#define EXT_SENS_DATA_00   0x49
#define EXT_SENS_DATA_01   0x4A
#define EXT_SENS_DATA_02   0x4B
#define EXT_SENS_DATA_03   0x4C
#define EXT_SENS_DATA_04   0x4D
#define EXT_SENS_DATA_05   0x4E
#define EXT_SENS_DATA_06   0x4F
#define EXT_SENS_DATA_07   0x50
#define EXT_SENS_DATA_08   0x51
#define EXT_SENS_DATA_09   0x52
#define EXT_SENS_DATA_10   0x53
#define EXT_SENS_DATA_11   0x54
#define EXT_SENS_DATA_12   0x55
#define EXT_SENS_DATA_13   0x56
#define EXT_SENS_DATA_14   0x57
#define EXT_SENS_DATA_15   0x58
#define EXT_SENS_DATA_16   0x59
#define EXT_SENS_DATA_17   0x5A
#define EXT_SENS_DATA_18   0x5B
#define EXT_SENS_DATA_19   0x5C
#define EXT_SENS_DATA_20   0x5D
#define EXT_SENS_DATA_21   0x5E
#define EXT_SENS_DATA_22   0x5F
#define EXT_SENS_DATA_23   0x60
#define MOT_DETECT_STATUS  0x61
#define I2C_SLV0_DO        0x63 
#define I2C_SLV1_DO        0x64
#define I2C_SLV2_DO        0x65
#define I2C_SLV3_DO        0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL    0x69
#define USER_CTRL          0x6A
#define PWR_MGMT_1         0x6B
#define PWR_MGMT_2         0x6C
#define BANK_SEL           0x6D
#define MEM_START_ADDR     0x6E
#define MEM_R_W            0x6F
#define DMP_CFG_1          0x70
#define DMP_CFG_2          0x71
#define FIFO_COUNTH        0x72
#define FIFO_COUNTL        0x73
#define FIFO_R_W           0x74
#define WHO_AM_I           0x75

// misc subroutines
void Var_Delay(unsigned long var_delay_time){ // variable delay (recommended for sound)
    unsigned long i;
    for(i = var_delay_time*100000; i != 0; i--) // delay
    i=i;
}

void Short_Delay(unsigned int var_delay_time){ // shorter variable delay (delay of 1 is about 10 us)
    unsigned int i;
    for(i = var_delay_time; i != 0; i--) // delay
    i=i;
}

// gamepad/controls subroutines
void Controller_Init(){ // init pins for interfacing with ext Famicom controller
    LATBbits.LATB5 = 0;   // init clock low
    LATBbits.LATB7 = 0;   // init latch low
    TRISBbits.TRISB5 = 0; // clock (output)
    TRISBbits.TRISB6 = 1; // data (input)
    TRISBbits.TRISB7 = 0; // latch (output)
}

void Controller_Read(){ // read Famicom/NES controller input
        // latch controller buttons
        LATBbits.LATB7 = 1;
        Short_Delay(1);
        button_A = !PORTBbits.RB6; // read A button
        LATBbits.LATB7 = 0;
        Short_Delay(1);
        // clock controller shift register
        LATBbits.LATB5 = 1;
        Short_Delay(1);
        button_B = !PORTBbits.RB6; // read B button
        LATBbits.LATB5 = 0;
        Short_Delay(1);
        LATBbits.LATB5 = 1;
        Short_Delay(1);
        button_Select = !PORTBbits.RB6; // read select button
        LATBbits.LATB5 = 0;
        Short_Delay(1);
        LATBbits.LATB5 = 1;
        Short_Delay(1);
        button_Start = !PORTBbits.RB6; // read start button
        LATBbits.LATB5 = 0;
        Short_Delay(1);
        LATBbits.LATB5 = 1;
        Short_Delay(1);
        button_Up = !PORTBbits.RB6; // read up button
        LATBbits.LATB5 = 0;
        Short_Delay(1);
        LATBbits.LATB5 = 1;
        Short_Delay(1);
        button_Down = !PORTBbits.RB6; // read down button
        LATBbits.LATB5 = 0;
        Short_Delay(1);
        LATBbits.LATB5 = 1;
        Short_Delay(1);
        button_Left = !PORTBbits.RB6; // read left button
        LATBbits.LATB5 = 0;
        Short_Delay(1);
        LATBbits.LATB5 = 1;
        Short_Delay(1);
        button_Right = !PORTBbits.RB6; // read right button
        LATBbits.LATB5 = 0;
        // combine button bits into compact "buttons" variable
        buttons = (button_A << 7) + (button_B << 6) + (button_Select << 5) +
                  (button_Start << 4) + (button_Up << 3) + (button_Down << 2) +
                  (button_Left << 1) + (button_Right);
        // correct readings if controller is detected to be disconnected
        // (test: if UP and DOWN both read in as true. Impossible on D-pad)
        if(button_Up && button_Down){
            buttons = 0;
            button_A = 0;
            button_B = 0;
            button_Select = 0;
            button_Start = 0;
            button_Up = 0;
            button_Down = 0;
            button_Left = 0;
            button_Right = 0;
            isPadConnected = 0;
        }
        else
            isPadConnected = 1;
}

void Timer1_Init(){
    T1CON = 0;
    TMR1 = 0;
    T1CONbits.TON = 1;      // Timer ON
    T1CONbits.TCKPS = 0;    // Set timer prescale bits
    T1CONbits.TCS = 0;      // Internal clock source (Fosc/2)
    T1CONbits.TGATE = 0;    // Gated time accumulation OFF
    IPC0bits.T1IP = 3;      // Timer1 interrupt priority = 3
    IFS0bits.T1IF= 0;       // clear Timer1 interrupt flag
    IEC0bits.T1IE = 1;      // enable Timer1 interrupts
}

void Timer2_Init(){
    T2CON = 0;
    TMR2 = 0;
    T2CONbits.T32 = 0;      // 16-bit timer mode
    T2CONbits.TCKPS = 3;    // Set timer prescale bits
    T2CONbits.TCS = 0;      // Internal clock source (Fosc/2)
    T2CONbits.TGATE = 0;    // Gated time accumulation OFF
    PR2 = 0xFF;             // Period
    T2CONbits.TON = 1;      // Turn on Timer2
    IPC1bits.T2IP = 3;      // Interrupt priority = 3
    IFS0bits.T2IF = 0;      // Clear interrupt flag
    IEC0bits.T2IE = 1;      // Enable interrupts (on timer 2)
}

void Timer23_Init(){ // for use with reading user input
    T2CON = 0;
    T3CON = 0;
    TMR2 = 0;
    TMR3 = 0;
    T2CONbits.T32 = 1;      // 32-bit timer (combined timers 2 and 3)
    T2CONbits.TCKPS = 0;    // Set timer prescale bits
    T2CONbits.TCS = 0;      // Internal clock source (Fosc/2)
    T2CONbits.TGATE = 0;    // Gated time accumulation OFF
    if(CONTROLLER_READ_60HZ){   // 60 Hz read
        PR3 = 0x0002;           // Period MSB
        PR2 = 0x4BA0;           // Period LSB
    }
    else{                       // 30 Hz read
        PR3 = 0x0004;           // Period MSB
        PR2 = 0x9740;           // Period LSB
    }
    T2CONbits.TON = 1;      // Turn on timer 2/3
    IPC2bits.T3IP = 3;      // Interrupt priority = 3
    IFS0bits.T3IF = 0;      // Clear interrupt flag
    IEC0bits.T3IE = 1;      // Enable interrupts (on timer 3)
}

void Gyro_Init(){ // initialize gyro/accelerometer
    // Init I2C module
    I2C1BRG = 37;           // Baud rate = 200 kHz
    I2C1CON = 0x1200;       // 7-bit address mode
	I2C1RCV = 0x0000;       // zero buffer
	I2C1TRN = 0x0000;       // zero buffer
	I2C1CON = 0x9200;       // enable I2C module
    // Init MPU6050
    __delay_ms(100);
    // Data rate config
    StartI2C();
    WriteI2C(MPU6050_ADDR<<1 | I2C_WRITE);
    IdleI2C();
    WriteI2C(SMPLRT_DIV);
    IdleI2C();
    WriteI2C(0x07);
    IdleI2C();
    StopI2C();
    // CLK source config
    StartI2C();
    WriteI2C(MPU6050_ADDR<<1 | I2C_WRITE);
    IdleI2C();
    WriteI2C(PWR_MGMT_1);
    IdleI2C();
    WriteI2C(0x01);
    IdleI2C();
    StopI2C();
    // DLPF config
    StartI2C();
    WriteI2C(MPU6050_ADDR<<1 | I2C_WRITE);
    IdleI2C();
    WriteI2C(CONFIG);
    IdleI2C();
    WriteI2C(0x00);
    IdleI2C();
    StopI2C();
    // ACCEL (FSR= +-2g)
    StartI2C();
    WriteI2C(MPU6050_ADDR<<1 | I2C_WRITE);
    IdleI2C();
    WriteI2C(ACCEL_CONFIG);
    IdleI2C();
    WriteI2C(0x00);
    IdleI2C();
    StopI2C();
    // GYRO (FSR= +-2000d/s)
    StartI2C();
    WriteI2C(MPU6050_ADDR<<1 | I2C_WRITE);
    IdleI2C();
    WriteI2C(GYRO_CONFIG);
    IdleI2C();
    WriteI2C(0x18);
    IdleI2C();
    StopI2C();
    // Data Ready Interrupt enable
    StartI2C();
    WriteI2C(MPU6050_ADDR<<1 | I2C_WRITE);
    IdleI2C();
    WriteI2C(INT_ENABLE);
    IdleI2C();
    WriteI2C(0x01);
    IdleI2C();
    StopI2C();
}

void Gyro_I2C_Read(){ // read all values from gyroscope/accelerometer
    StartI2C();
    WriteI2C(MPU6050_ADDR<<1 | I2C_WRITE);
    IdleI2C();
    WriteI2C(ACCEL_XOUT_H);
    IdleI2C();
    RestartI2C();
    WriteI2C(MPU6050_ADDR<<1 | I2C_READ);
    IdleI2C();
    if(FIX_ORIENTATION){ // default happens to be correct orientation
        Ax = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);    // Accel x-axis
        Ay = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);    // Accel y-axis
        Az = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);    // Accel z-axis
        T  = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);    // Temperature
        Gx = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);    // Gyro x-axis
        Gy = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);    // Gyro y-axis
        Gz = ((int)I2C_Read(0)<<8) | (int)I2C_Read(1);    // Gyro z-axis
    }
    else{
        Ax = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);    // Accel x-axis
        Ay = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);    // Accel y-axis
        Az = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);    // Accel z-axis
        T  = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);    // Temperature
        Gx = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);    // Gyro x-axis
        Gy = ((int)I2C_Read(0)<<8) | (int)I2C_Read(0);    // Gyro y-axis
        Gz = ((int)I2C_Read(0)<<8) | (int)I2C_Read(1);    // Gyro z-axis
    }
    StopI2C();
}

void ADC_Init(){    // for use with analog thumbstick
    AD1PCFGbits.PCFG0 = 0;  // AN0 analog in
    AD1PCFGbits.PCFG1 = 0;  // AN1 analog in
    TRISAbits.TRISA0 = 1;   // AN0 input
    TRISAbits.TRISA1 = 1;   // AN1 input
    AD1CON2bits.VCFG = 0;   // Vref = Vdd/Vss
    AD1CON3bits.ADCS = 0;   // AD conversion clock = Fcy
    AD1CON1bits.ASAM = 1;   // Begin sampling immediately after conversion
    AD1CON1bits.SSRC = 7;   // Auto convert samples
    AD1CON3bits.SAMC = 15;  // Auto sample time bits
    AD1CON1bits.FORM = 0;   // Return integer format
    AD1CON2bits.SMPI = 0;   // Interrupt at conversion done
    AD1CHSbits.CH0SA = 0;   // Mux A <- AN0
    AD1CHSbits.CH0SB = 1;   // Mux B <- AN1
    AD1CON2bits.ALTS = 0;   // Alternate between 2 mux settings for conversions
    AD1CON1bits.ADON = 1;   // Turn on ADC
}

void OBC_Read(){ // read on-board controls with or without gyro enabled
    if(read_y_axis){ // reading y axis
        AD1CON1bits.SAMP = 0; // start conversion
        while(!AD1CON1bits.DONE);
        AD1CHSbits.CH0SA = 0; // Mux A <- AN0 (x)
        thumb_y = ADC1BUF0;
        read_y_axis = false;
    }
    else{ // reading x axis
        AD1CON1bits.SAMP = 0; // start conversion
        while(!AD1CON1bits.DONE);
        AD1CHSbits.CH0SA = 1; // Mux A <- AN1 (y)
        thumb_x = ADC1BUF0;
        read_y_axis = true;
    }
    if(thumb_x > DEADZONEHI)
        button_Right = 1;
    else
        button_Right = 0;
    if(thumb_x < DEADZONELO)
        button_Left = 1;
    else
        button_Left = 0;
    if(thumb_y > DEADZONEHI)
        button_Up = 1;
    else 
        button_Up = 0;
    if(thumb_y < DEADZONELO)
        button_Down = 1;
    else
        button_Down = 0;
    if(PORTAbits.RA2)
        button_A = 1;
    else
        button_A = 0;
    if(PORTAbits.RA3)
        button_B = 1;
    else
        button_B = 0;
    if(USE_GYRO){ // gyro enabled, OBC Start + Select disabled
        Gyro_I2C_Read();
        button_Start = 0;
        button_Select = 0;
    }
    else{ // gyro disabled, read OBC Start and Select buttons
        if(PORTBbits.RB8)
            button_Start = 1;
        else
            button_Start = 0;
        if(PORTBbits.RB9)
            button_Select = 1;
        else
            button_Select = 0;
    }
    // combine button bits into compact "buttons" variable
    buttons = (button_A << 7) + (button_B << 6) + (button_Select << 5) +
              (button_Start << 4) + (button_Up << 3) + (button_Down << 2) +
              (button_Left << 1) + (button_Right);
}

void OBC_Init(){ // initialize OBC (on-board controls)
    if(USE_GYRO){   // disables on-board Start and Select buttons
        Gyro_Init();
        ADC_Init();     // initialize analog stick (direction controls)
        TRISAbits.TRISA2 = 1;   // RA2 input (A)
        TRISAbits.TRISA3 = 1;   // RA3 input (B)
    }
    else{           // disables gyro controls
        ADC_Init();     // initialize analog stick (direction controls)
        TRISAbits.TRISA2 = 1;   // RA2 input (A)
        TRISAbits.TRISA3 = 1;   // RA3 input (B)
        TRISBbits.TRISB8 = 1;   // RB8 input (Start)    [shared w/ pin I2C CLK]
        TRISBbits.TRISB9 = 1;   // RB9 input (Select)   [shared w/ pin I2C DA]
    }

}

void Master_Controller_Init(){ // call all controller init functions
    Controller_Init();  // init port IO
    OBC_Init();         // init OBC
    Timer23_Init();     // init polling timer
}

// sound subroutines
void DAC_Init(){
    // unlock control registers
    __builtin_write_OSCCONL(OSCCON & ~0x40);
    
    // configure pins for SPI1
    TRISBbits.TRISB2 = 0;       // CS output direction
    LATBbits.LATB2 = 1;         // initialize CS high
    RPOR1bits.RP2R = 9;         // RP2 = SP1 SS output
    RPOR1bits.RP3R = 8;         // RP3 = SP1 clock out
    RPOR2bits.RP4R = 7;         // RP4 = SP1 data out
    
    // configure SPI1 control register
    SPI1CON1bits.MSTEN = 1;     // SPI1 master mode
    SPI1CON1bits.DISSCK = 0;    // use internal SPI clock
    SPI1CON1bits.MODE16 = 0;    // 8-bit communication mode
    SPI1CON1bits.CKP = 0;       // clock idle state is low
    SPI1CON1bits.CKE = 1;       // change output on clock falling edge
    SPI1CON1bits.PPRE = 3;      // primary prescaler settings
    SPI1CON1bits.SPRE = 7;      // secondary prescaler settings
    
    // enable SPI1 via status register
    SPI1STATbits.SPIROV = 0;    // clear RX overflow flag
    SPI1STATbits.SPIEN = 1;     // enable SPI1
    
    // lock control registers
    __builtin_write_OSCCONL(OSCCON | 0x40);
}

void Send_SPI_Sound(unsigned short* const waveform, unsigned short vector_length, unsigned int freq_delay, unsigned int tone_duration){
    // for use with 10-bit external DAC, SPI1
    unsigned char dummy; // to receive garbage data back in SPI
    unsigned short sample;
    unsigned short x, i, j;
    for(x = 0; x < tone_duration; x++){
        for(i = 0; i < vector_length; i++){
            sample = waveform[i];
            LATBbits.LATB2 = 0; // pull DAC CS down to begin TX
            SPI1BUF = DAC_CTRL_CODE | ((sample & 0b1111000000) >> 6);
            while(!(SPI1STAT & 1)); // wait for buffer ready
            dummy = SPI1BUF; // read in dummy data
            while(SPI1STAT & 1); // wait for buffer ready
            SPI1BUF = (sample & 0b0000111111) << 2;
            while(!(SPI1STAT & 1)); // wait for buffer ready
            dummy = SPI1BUF; // read in dummy data
            while(SPI1STAT & 1); // wait for buffer ready
            LATBbits.LATB2 = 1; // pull DAC CS up to end TX
            // frequency delay
            for(j = freq_delay; j != 0; j--)
                j=j;
        }
    }
}

// display/SD slot subroutines
void LCD_SD_Init(){
    // unlock control registers
    __builtin_write_OSCCONL(OSCCON & ~0x40);
    
    // configure pins for SPI2
    TRISBbits.TRISB12 = 0;       // LCD CS output direction
    LATBbits.LATB12 = 1;         // initialize LCD CS high
    TRISBbits.TRISB11 = 0;       // SD CS output direction
    LATBbits.LATB11 = 1;         // initialize SD CS high

    RPOR6bits.RP12R = 12;        // RP12 = SPI2 SS output (LCD)
    RPOR5bits.RP11R = 12;        // RP11 = SPI2 SS output (SD)
    
    RPOR7bits.RP15R = 11;        // RP15 = SPI2 clock out
    RPINR22bits.SDI2R = 14;      // RP14 = SPI2 data in (MISO)
    RPOR6bits.RP13R = 10;        // RP13 = SPI2 data out (MOSI)
    
    // configure other pins for interfacing
    LATBbits.LATB10 = 0;         // initialize Data/Command low (low command)
    TRISBbits.TRISB10 = 0;       // LCD Data/Command (output)
    
    // configure SPI2 control register
    SPI2CON1bits.MSTEN = 1;     // SPI2 master mode
    SPI2CON1bits.DISSCK = 0;    // use internal SPI clock
    SPI2CON1bits.MODE16 = 0;    // 8-bit communication mode
    SPI2CON1bits.CKP = 0;       // clock idle state is low
    SPI2CON1bits.CKE = 1;       // change output on clock falling edge
    SPI2CON1bits.PPRE = 0b11;   // SCK 1:1 primary prescale
    SPI2CON1bits.SPRE = 0b111;  // SCK 1:1 primary prescale
    
    // set SPI2 interrupt event trigger
    SPI2STATbits.SISEL = 0b111; // interrupt when TX buffer is written to
    
    // enable SPI2 via status register
    SPI2STATbits.SPIROV = 0;    // clear RX overflow flag
    SPI2STATbits.SPIEN = 1;     // enable SPI2
    
    // SPI2 interrupt config
    IPC8bits.SPI2IP = 5;    // SPI2 interrupt priority 5
    IFS2bits.SPI2IF = 0;    // clear SPI2 interrupt flag
    IEC2bits.SPI2IE = 0;    // DISABLE SPI2 interrupts
    
    // lock control registers
    __builtin_write_OSCCONL(OSCCON | 0x40);
}


// main
int main(int argc, char** argv){
    
    OSCTUN = 0b011111; // Max osc freq with PLL
    
    AD1PCFG = 0xFFFF; // initialize all analog pins as digital IO
    
    INTCON1bits.NSTDIS = 1; // interrupt nesting disabled
    
    // used for writing score to screen in Super Mario mode
    char* intString[26] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", 
                            "10", "11", "12", "13", "14", "15", "16", "17",
                            "18", "19", "20", "21", "22", "23", "24", "25"};
    
    // used for sound engine
    static unsigned short silence[2] = {0, 0};
    static unsigned short sineWave128[128] = { 
                            0x200,0x219,0x232,0x24b,0x263,0x27c,0x294,0x2ac,
                            0x2c3,0x2da,0x2f1,0x306,0x31c,0x330,0x344,0x357,
                            0x369,0x37a,0x38b,0x39a,0x3a9,0x3b6,0x3c3,0x3ce,
                            0x3d8,0x3e1,0x3e9,0x3f0,0x3f5,0x3f9,0x3fd,0x3fe,
                            0x3ff,0x3fe,0x3fd,0x3f9,0x3f5,0x3f0,0x3e9,0x3e1,
                            0x3d8,0x3ce,0x3c3,0x3b6,0x3a9,0x39a,0x38b,0x37a,
                            0x369,0x357,0x344,0x330,0x31c,0x306,0x2f1,0x2da,
                            0x2c3,0x2ac,0x294,0x27c,0x263,0x24b,0x232,0x219,
                            0x200,0x1e6,0x1cd,0x1b4,0x19c,0x183,0x16b,0x153,
                            0x13c,0x125,0x10e,0xf9,0xe3,0xcf,0xbb,0xa8,
                            0x96,0x85,0x74,0x65,0x56,0x49,0x3c,0x31,
                            0x27,0x1e,0x16,0xf,0xa,0x6,0x2,0x1,
                            0x0,0x1,0x2,0x6,0xa,0xf,0x16,0x1e,
                            0x27,0x31,0x3c,0x49,0x56,0x65,0x74,0x85,
                            0x96,0xa8,0xbb,0xcf,0xe3,0xf9,0x10e,0x125,
                            0x13c,0x153,0x16b,0x183,0x19c,0x1b4,0x1cd,0x1e6};
    
    // init DAC
    DAC_Init();
                
    // init ext gamepad
    Master_Controller_Init();
    
    // init display & SD card
    LCD_SD_Init();
    init_ILI9340();                     // init LCD
    Adafruit_GFX(240, 320);             // init GFX library with default font
    
    // fill screen with white if QUICKBOOT disabled
    if(!QUICKBOOT)
        fillScreen(ILI9340_WHITE);

    // menu/BIOS related variables and objects
    struct Arrow mainArrow;      // arrow/cursor declaration
    struct Sprite player;        // player sprite
    player.draw = false;
    struct Sprite bullet;        // Bullet enemy sprite
    bullet.draw = false;
    long dPadDelayCounter = 0;   // cooldown software timer
    short menuSelection = 0;     // menu selection value
    bool isWarningDisplayed = 0; // misc bool for menus
    int16_t tmp_x, tmp_y, tmp, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7 = 0; // temporary coordinates for misc purposes
    uint32_t tmp8, tmp9;
    short enemy_type, enemy_height;
    bool contextTrigger = 0;     // misc bool
    int misc_timer = 0;          // misc timer variable
    
    // for gyro reading display (gyro test)
    char bufferAx[10] = {0};
    char bufferAy[10] = {0};
    char bufferAz[10] = {0};
    char bufferGx[10] = {0};
    char bufferGy[10] = {0};
    char bufferGz[10] = {0};
    char bufferT[10]  = {0};
    
    while(1){                                                                   // SYSTEM STATE VIEW ///////////////////////////
        
        
        while(current_state == STATE_SYSTEM_BOOT){                              // STATE_SYSTEM_BOOT
            // draw animated boot logo
            fillRect(10, 100, 120, 20, ILI9340_RED);
            fillRect(10, 130, 120, 20, ILI9340_GREEN);
            fillRect(10, 160, 120, 20, ILI9340_BLUE);
            fillRect(10, 190, 120, 20, ILI9340_YELLOW);
            fillRect(10, 190, 120, 20, ILI9340_CYAN);
            fillRect(10, 130, 120, 20, ILI9340_MAGENTA);
            fillRect(10, 100, 120, 20, ILI9340_YELLOW);
            fillRect(10, 190, 120, 20, ILI9340_GREEN);
            fillRect(10, 160, 120, 20, ILI9340_RED);
            fillRect(10, 160, 120, 20, ILI9340_BLUE);
            drawFCpad(50, 125, 7);
            // play boot theme and display my name
            Send_SPI_Sound(sineWave128, 128, 65, 15);
            Send_SPI_Sound(sineWave128, 128, 60, 15);
            Send_SPI_Sound(sineWave128, 128, 55, 15);
            Send_SPI_Sound(sineWave128, 128, 50, 15);
            Send_SPI_Sound(sineWave128, 128, 45, 15);
            Send_SPI_Sound(sineWave128, 128, 40, 15);
            Send_SPI_Sound(sineWave128, 128, 35, 15);
            setTextColor(ILI9340_BLACK);
            setTextSize(1);
            setCursor(50, 230);
            WriteString("2021 Louis Eric DiGioia");
            Var_Delay(4);
            Send_SPI_Sound(sineWave128, 128, 28, 250);
            Send_SPI_Sound(sineWave128, 128, 38, 200);
            Var_Delay(5);
            // erase boot logo
            setCursor(50, 230);
            EraseString("2021 Louis Eric DiGioia", ILI9340_WHITE);
            fillRect(50, 125, 147, 63, ILI9340_RED);
            fillRect(50, 125, 147, 63, ILI9340_WHITE);
            fillRect(10, 100, 120, 20, ILI9340_WHITE);
            fillRect(10, 130, 120, 20, ILI9340_WHITE);
            fillRect(10, 160, 120, 20, ILI9340_WHITE);
            fillRect(10, 190, 120, 20, ILI9340_WHITE);
            Var_Delay(1);
            current_state = STATE_SDCARD_CHECK;
        }
        
        
        
        
        while(current_state == STATE_SDCARD_CHECK){                             // STATE_SDCARD_CHECK
            setTextColor(ILI9340_BLACK);
            setTextSize(1);
            setCursor(50, 150);
            WriteString("No SD card detected!");
            Send_SPI_Sound(sineWave128, 128, 20, 6);
            setCursor(50, 170);
            WriteString("Booting to BIOS...");
            Var_Delay(13);
            setCursor(50, 150);
            EraseString("No SD card detected!", ILI9340_WHITE);
            setCursor(50, 170);
            EraseString("Booting to BIOS...", ILI9340_WHITE);
            current_state = STATE_BIOS_MAINMENU;
        }
        
        
        
        
        while(current_state == STATE_BIOS_MAINMENU){                            // STATE_BIOS_MAINMENU
            if(newState){
                //draw BIOS menu
                isWarningDisplayed = 0;
                if(sys_lang == LANG_EN){
                    setTextColor(ILI9340_BLACK);
                    setTextSize(3);
                    setCursor(30, 20);
                    WriteString("BIOS MENU");
                    setTextSize(2);
                    setCursor(0, 82);
                    WriteString("  System Test\n\n  System Language\n\n  Boot SD Card\n\n  Game Demo");
                }
                else if(sys_lang == LANG_ES){
                    setTextColor(ILI9340_BLACK);
                    setTextSize(3);
                    setCursor(30, 20);
                    WriteString("MENU BIOS");
                    setTextSize(2);
                    setCursor(0, 82);
                    WriteString("  Prueba del Sistema\n\n  Idioma del Sistema\n\n  Arrancar SD Card\n\n  Demo Game");
                }
                RemoveArrow(&mainArrow); // remove arrow in case already initialized
                ArrowDefine(&mainArrow, 10, 80, ILI9340_BLUE, ILI9340_WHITE, 2);
                newState = 0;
                menuSelection = 0;
            }
            // Move cursor down
            if(button_Down && !dPadDelayCounter && (menuSelection != 3)){
                dPadDelayCounter = 60000;
                Send_SPI_Sound(sineWave128, 128, 70, 5);
                MoveArrow(&mainArrow, 0, 32);
                menuSelection++;
            }
            // Move cursor up
            else if(button_Up && !dPadDelayCounter && menuSelection){
                dPadDelayCounter = 60000;
                Send_SPI_Sound(sineWave128, 128, 70, 5);
                MoveArrow(&mainArrow, 0, -32);
                menuSelection--;
            }
            // Select menu item (start or select or A)
            else if((button_A || button_Select || button_Start) && !dPadDelayCounter){
                switch(menuSelection){
                    case 0:
                        current_state = STATE_SYS_TEST_MENU;
                        newState = 1;
                        break;
                    case 1:
                        current_state = STATE_SYS_LANG_MENU;
                        newState = 1;
                        break;
                    case 2:
                        current_state = STATE_SDCARD_CHECK;
                        newState = 1;
                        break;
                    case 3:
                        current_state = STATE_SUPER_MARIO;
                        newState = 1;
                        break;
                    default:
                        current_state = STATE_SYS_ERROR;
                        newState = 1;
                }
            }
            // Decrement selection delay counter
            if(dPadDelayCounter)
                dPadDelayCounter--;
            // Display controller disconnected status message on bottom right
            if(!isPadConnected && !isWarningDisplayed){
                setTextColor(ILI9340_RED);
                setTextSize(1);
                setCursor(90, 300);
                WriteString("CONTROLLER DISCONNECTED");
                isWarningDisplayed = 1;
                setTextColor(ILI9340_BLACK);
            } else if(isWarningDisplayed && isPadConnected){
                setTextSize(1);
                setCursor(90, 300);
                EraseString("CONTROLLER DISCONNECTED", ILI9340_WHITE);
                isWarningDisplayed = 0;
            }
            // Erase menu items if about to switch states
            if(current_state != STATE_BIOS_MAINMENU){
                RemoveArrow(&mainArrow);
                if(sys_lang == LANG_EN){
                    setTextSize(3);
                    setCursor(30, 20);
                    EraseString("BIOS MENU", ILI9340_WHITE);
                    setTextSize(2);
                    setCursor(0, 82);
                    EraseString("  System Test\n\n  System Language\n\n  Boot SD Card\n\n  Game Demo", ILI9340_WHITE);
                }
                else if(sys_lang == LANG_ES){
                    setTextSize(3);
                    setCursor(30, 20);
                    EraseString("MENU BIOS", ILI9340_WHITE);
                    setTextSize(2);
                    setCursor(0, 82);
                    EraseString("  Prueba del Sistema\n\n  Idioma del Sistema\n\n  Arrancar SD Card\n\n  Demo Game", ILI9340_WHITE);
                }
            }
        }
        
        
        
        
        while(current_state == STATE_SYS_TEST_MENU){                            // STATE_SYS_TEST_MENU
                if(newState){
                //draw test menu
                if(sys_lang == LANG_EN){
                    setTextColor(ILI9340_BLACK);
                    setTextSize(3);
                    setCursor(20, 20);
                    WriteString("SYSTEM TEST");
                    setTextSize(2);
                    setCursor(0, 82);
                    WriteString("  Controller Test\n\n  Display Test\n\n  Sound Test\n\n  Gyro Test");
                }
                else if(sys_lang == LANG_ES){
                    setTextColor(ILI9340_BLACK);
                    setTextSize(3);
                    setCursor(20, 20);
                    WriteString("PR. SISTEMA");
                    setTextSize(2);
                    setCursor(0, 82);
                    WriteString("  Pr. Controlador\n\n  Prueba de Pantalla\n\n  Prueba de Sonido\n\n  Prueba de Gyro");
                }
                ArrowDefine(&mainArrow, 10, 80, ILI9340_BLUE, ILI9340_WHITE, 2);
                isWarningDisplayed = 0;
                newState = 0;
                menuSelection = 0;
            }
            // Move cursor down
            if(button_Down && !dPadDelayCounter && (menuSelection != 3)){
                dPadDelayCounter = 60000;
                Send_SPI_Sound(sineWave128, 128, 70, 5);
                MoveArrow(&mainArrow, 0, 32);
                menuSelection++;
            }
            // Move cursor up
            else if(button_Up && !dPadDelayCounter && menuSelection){
                dPadDelayCounter = 60000;
                Send_SPI_Sound(sineWave128, 128, 70, 5);
                MoveArrow(&mainArrow, 0, -32);
                menuSelection--;
            }
            // Select menu item (start or select or A)
            else if((button_A || button_Select || button_Start) && !dPadDelayCounter){
                switch(menuSelection){
                    case 0:
                        current_state = STATE_SYS_CONT_TEST;
                        newState = 1;
                        break;
                    case 1:
                        current_state = STATE_SYS_SCRN_TEST;
                        newState = 1;
                        break;
                    case 2:
                        current_state = STATE_SYS_SND_TEST;
                        newState = 1;
                        break;
                    case 3:
                        current_state = STATE_SYS_GYRO_TEST;
                        newState = 1;
                        break;
                    default:
                        current_state = STATE_SYS_ERROR;
                        newState = 1;
                }
            }
            // Navigate to previous menu
            else if(button_B){
                current_state = STATE_BIOS_MAINMENU;
                newState = 1;
            }
            // Decrement selection delay counter
            if(dPadDelayCounter)
                dPadDelayCounter--;
            // Display controller disconnected status message on bottom right
            if(!isPadConnected && !isWarningDisplayed){
                setTextColor(ILI9340_RED);
                setTextSize(1);
                setCursor(90, 300);
                WriteString("CONTROLLER DISCONNECTED");
                isWarningDisplayed = 1;
                setTextColor(ILI9340_BLACK);
            } else if(isWarningDisplayed && isPadConnected){
                setTextSize(1);
                setCursor(90, 300);
                EraseString("CONTROLLER DISCONNECTED", ILI9340_WHITE);
                isWarningDisplayed = 0;
            }
            // Erase menu items if about to switch states
            if(current_state != STATE_SYS_TEST_MENU){
                RemoveArrow(&mainArrow);
                if(sys_lang == LANG_EN){
                    setTextSize(3);
                    setCursor(20, 20);
                    EraseString("SYSTEM TEST", ILI9340_WHITE);
                    setTextSize(2);
                    setCursor(0, 82);
                    EraseString("  Controller Test\n\n  Display Test\n\n  Sound Test\n\n  Gyro Test", ILI9340_WHITE);
                }
                else if(sys_lang == LANG_ES){
                    setTextSize(3);
                    setCursor(20, 20);
                    EraseString("PR. SISTEMA", ILI9340_WHITE);
                    setTextSize(2);
                    setCursor(0, 82);
                    EraseString("  Pr. Controlador\n\n  Prueba de Pantalla\n\n  Prueba de Sonido\n\n  Prueba de Gyro", ILI9340_WHITE);
                }
            }
        }
        
        
        
        
        while(current_state == STATE_SYS_LANG_MENU){                            // STATE_SYS_LANG_MENU
                if(newState){
                //draw language menu
                if(sys_lang == LANG_EN){
                    tmp = 0; // record current lang
                    setTextColor(ILI9340_BLACK);
                    setTextSize(3);
                    setCursor(20, 20);
                    WriteString("LANGUAGES");
                    setTextSize(2);
                    setCursor(0, 82);
                    WriteString("  English\n\n  Espanol\n\n  Portugues\n\n  Francais");
                }
                else if(sys_lang == LANG_ES){
                    tmp = 1; // record current lang
                    setTextColor(ILI9340_BLACK);
                    setTextSize(3);
                    setCursor(20, 20);
                    WriteString("IDIOMAS");
                    setTextSize(2);
                    setCursor(0, 82);
                    WriteString("  English\n\n  Espanol\n\n  Portugues\n\n  Francais");
                }
                ArrowDefine(&mainArrow, 10, 80, ILI9340_BLUE, ILI9340_WHITE, 2);
                isWarningDisplayed = 0;
                newState = 0;
                menuSelection = 0;
            }
            // Move cursor down
            if(button_Down && !dPadDelayCounter && (menuSelection != 3)){
                dPadDelayCounter = 60000;
                Send_SPI_Sound(sineWave128, 128, 70, 5);
                MoveArrow(&mainArrow, 0, 32);
                menuSelection++;
            }
            // Move cursor up
            else if(button_Up && !dPadDelayCounter && menuSelection){
                dPadDelayCounter = 60000;
                Send_SPI_Sound(sineWave128, 128, 70, 5);
                MoveArrow(&mainArrow, 0, -32);
                menuSelection--;
            }
            // Select menu item (start or select or A)
            else if((button_A || button_Select || button_Start) && !dPadDelayCounter){
                switch(menuSelection){
                    case 0: // English (default)
                        sys_lang = LANG_EN;
                        current_state = STATE_BIOS_MAINMENU;
                        newState = 1;
                        break;
                    case 1: // Spanish
                        sys_lang = LANG_ES;
                        current_state = STATE_BIOS_MAINMENU;
                        newState = 1;
                        break;
                    case 2: // Portuguese not implemented
                        sys_lang = LANG_PT;
                        current_state = STATE_SYS_ERROR;
                        newState = 1;
                        break;
                    case 3: // French not implemented
                        sys_lang = LANG_FR;
                        current_state = STATE_SYS_ERROR;
                        newState = 1;
                        break;
                    default:
                        current_state = STATE_SYS_ERROR;
                        newState = 1;
                }
            }
            // Navigate to previous menu
            else if(button_B){
                current_state = STATE_BIOS_MAINMENU;
                newState = 1;
            }
            // Display controller disconnected status message on bottom right
            if(!isPadConnected && !isWarningDisplayed){
                setTextColor(ILI9340_RED);
                setTextSize(1);
                setCursor(90, 300);
                WriteString("CONTROLLER DISCONNECTED");
                isWarningDisplayed = 1;
                setTextColor(ILI9340_BLACK);
            } else if(isWarningDisplayed && isPadConnected){
                setTextSize(1);
                setCursor(90, 300);
                EraseString("CONTROLLER DISCONNECTED", ILI9340_WHITE);
                isWarningDisplayed = 0;
            }
            // Decrement selection delay counter
            if(dPadDelayCounter)
                dPadDelayCounter--;
            // Erase menu items if about to switch states
            if(current_state != STATE_SYS_LANG_MENU){
                RemoveArrow(&mainArrow);
                if(tmp == 0){
                    setTextSize(3);
                    setCursor(20, 20);
                    EraseString("LANGUAGES", ILI9340_WHITE);
                    setTextSize(2);
                    setCursor(0, 82);
                    EraseString("  English\n\n  Espanol\n\n  Portugues\n\n  Francais", ILI9340_WHITE);
                }
                else if(tmp == 1){
                    setTextSize(3);
                    setCursor(20, 20);
                    EraseString("IDIOMAS", ILI9340_WHITE);
                    setTextSize(2);
                    setCursor(0, 82);
                    EraseString("  English\n\n  Espanol\n\n  Portugues\n\n  Francais", ILI9340_WHITE);
                }
            }
        }
        
        
        
        
        while(current_state == STATE_BOOT_SDCARD){                              // STATE_BOOT_SDCARD
            
        }
        
        
        
        
        while(current_state == STATE_SYS_CONT_TEST){                            // STATE_SYS_CONT_TEST
            if(newState){
                //draw gamepad graphic
                tmp_x = 25;
                tmp_y = 110;
                tmp = 9;
                drawFCpad(tmp_x, tmp_y, tmp);
                fillRect(tmp_x+(10*tmp-2), 0, tmp, tmp_y, ILI9340_BLACK);
                setTextColor(ILI9340_BLACK);
                setTextSize(1);
                setCursor(30, 220);
                WriteString("To exit: hold DOWN and B");
                misc_timer = 0;
                isWarningDisplayed = 0;
                newState = 0;
            }
            // Highlight pressed buttons on screen
            contextTrigger = 0; // set if a d-pad button is pressed
            if(button_A)
                fillRect(tmp_x+(17*tmp), tmp_y+(5*tmp), 2*tmp, 2*tmp, ILI9340_CYAN);
            else
                fillRect(tmp_x+(17*tmp), tmp_y+(5*tmp), 2*tmp, 2*tmp, ILI9340_BLACK);
            if(button_B)
                fillRect(tmp_x+(14*tmp), tmp_y+(5*tmp), 2*tmp, 2*tmp, ILI9340_CYAN);
            else
                fillRect(tmp_x+(14*tmp), tmp_y+(5*tmp), 2*tmp, 2*tmp, ILI9340_BLACK);
            if(button_Start)
                fillRect(tmp_x+(10*tmp), tmp_y+(5*tmp), 2*tmp, 1*tmp, ILI9340_CYAN);
            else
                fillRect(tmp_x+(10*tmp), tmp_y+(5*tmp), 2*tmp, 1*tmp, ILI9340_BLACK);
            if(button_Select)
                fillRect(tmp_x+(7*tmp), tmp_y+(5*tmp), 2*tmp, 1*tmp, ILI9340_CYAN);
            else
                fillRect(tmp_x+(7*tmp), tmp_y+(5*tmp), 2*tmp, 1*tmp, ILI9340_BLACK);
            if(button_Up){
                fillRect(tmp_x+(3*tmp), tmp_y+(4*tmp), 1*tmp, 1*tmp, ILI9340_CYAN);
                contextTrigger = 1;
            }
            else
                fillRect(tmp_x+(3*tmp), tmp_y+(4*tmp), 1*tmp, 1*tmp, ILI9340_BLACK);
            if(button_Down){
                fillRect(tmp_x+(3*tmp), tmp_y+(6*tmp), 1*tmp, 1*tmp, ILI9340_CYAN);
                contextTrigger = 1;
            }
            else
                fillRect(tmp_x+(3*tmp), tmp_y+(6*tmp), 1*tmp, 1*tmp, ILI9340_BLACK);
            if(button_Left){
                fillRect(tmp_x+(2*tmp), tmp_y+(5*tmp), 1*tmp, 1*tmp, ILI9340_CYAN);
                contextTrigger = 1;
            }
            else
                fillRect(tmp_x+(2*tmp), tmp_y+(5*tmp), 1*tmp, 1*tmp, ILI9340_BLACK);
            if(button_Right){
                fillRect(tmp_x+(4*tmp), tmp_y+(5*tmp), 1*tmp, 1*tmp, ILI9340_CYAN);
                contextTrigger = 1;
            }
            else
                fillRect(tmp_x+(4*tmp), tmp_y+(5*tmp), 1*tmp, 1*tmp, ILI9340_BLACK);
            if(contextTrigger) // highlight d-pad center
                fillRect(tmp_x+(3*tmp), tmp_y+(5*tmp), 1*tmp, 1*tmp, ILI9340_CYAN);
            else
                fillRect(tmp_x+(3*tmp), tmp_y+(5*tmp), 1*tmp, 1*tmp, ILI9340_BLACK);
            // Display controller disconnected status message on bottom right
            if(!isPadConnected && !isWarningDisplayed){
                setTextColor(ILI9340_RED);
                setTextSize(1);
                setCursor(90, 300);
                WriteString("CONTROLLER DISCONNECTED");
                isWarningDisplayed = 1;
                setTextColor(ILI9340_BLACK);
            } else if(isWarningDisplayed && isPadConnected){
                setTextSize(1);
                setCursor(90, 300);
                EraseString("CONTROLLER DISCONNECTED", ILI9340_WHITE);
                isWarningDisplayed = 0;
            }
            // handle exit condition: holding SELECT and START
            if(button_Down && button_B){
                misc_timer++;
            }
            else
                misc_timer = 0;
            // handle exiting state when exit condition is met
            if(misc_timer == 20){
                //erase gamepad and text
                setCursor(30, 220);
                EraseString("To exit: hold DOWN and B", ILI9340_WHITE);
                fillRect(tmp_x+(10*tmp-2), 0, tmp, tmp_y, ILI9340_WHITE);
                fillRect(tmp_x, tmp_y, 21*tmp, 9*tmp, ILI9340_WHITE);
                current_state = STATE_SYS_TEST_MENU;
                newState = 1;
            }
            
        }
        
        
        
        
        while(current_state == STATE_SYS_SCRN_TEST){                            // STATE_SYS_SCRN_TEST
            if(newState){
                misc_timer = 0;
                //draw screen test screen
                tmp = 10;  // used here for speed variable
                tmp2 = 5;  // used here to change square size
                tmp3 = 0;  // used here to cycle colors
                tmp4 = 10; // used here to remember square width and height
                setTextSize(2);
                setCursor(0, 100);
                WriteString("  A: Change color\n\n  B: Change size\n\n  Hold B: Exit");
                ArrowDefine(&mainArrow, 120, 160, colorCycle[tmp3], ILI9340_WHITE, tmp2);
                isWarningDisplayed = 0;
                newState = 0;
            }
            // change color
            if(button_A && !dPadDelayCounter){
                dPadDelayCounter = 50000;
                tmp3++;
                if(tmp3 == 18){ tmp3 = 0; }
                mainArrow.arrow_color = colorCycle[tmp3];
                MoveArrow(&mainArrow, 0, 0);
            }
            // change size
            else if(button_B && !dPadDelayCounter){
                dPadDelayCounter = 50000;
                tmp2++;     // update size number
                tmp4 += 10; // update pixel dimensions
                if(tmp2 == 8){ tmp2 = 5; tmp4 = 10; }
                RemoveArrow(&mainArrow);
                ArrowDefine(&mainArrow, mainArrow.curr_x, mainArrow.curr_y, colorCycle[tmp3], ILI9340_WHITE, tmp2);
            }
            // move sprite up
            if(button_Up){
                MoveArrow(&mainArrow, 0, -tmp);
            }
            // move sprite down
            else if(button_Down){
                MoveArrow(&mainArrow, 0, tmp);
            }
            // move sprite left
            if(button_Left){
                MoveArrow(&mainArrow, -tmp, 0);
            }
            // move sprite right
            else if(button_Right){
                MoveArrow(&mainArrow, tmp, 0);
            }
            // handle collisions with screen border
            if(mainArrow.curr_x > SCREEN_WIDTH-tmp4){
                mainArrow.curr_x = SCREEN_WIDTH-tmp4;
                MoveArrow(&mainArrow, 0, 0);
            }
            else if(mainArrow.curr_x < 0){
                mainArrow.curr_x = 0;
                MoveArrow(&mainArrow, 0, 0);
            }
            if(mainArrow.curr_y > SCREEN_HEIGHT-tmp4){
                mainArrow.curr_y = SCREEN_HEIGHT-tmp4;
                MoveArrow(&mainArrow, 0, 0);
            }
            else if(mainArrow.curr_y < 0){
                mainArrow.curr_y = 0;
                MoveArrow(&mainArrow, 0, 0);
            }
            // exit screen test
            if(button_B){
                misc_timer++;
            }
            else
                misc_timer = 0;
            if(misc_timer == 30000){
                misc_timer = 0;
                current_state = STATE_SYS_TEST_MENU;
                newState = 1;
            }
            // Decrement cooldown counter
            if(dPadDelayCounter)
                dPadDelayCounter--;
            // Display controller disconnected status message on bottom right
            if(!isPadConnected && !isWarningDisplayed){
                setTextColor(ILI9340_RED);
                setTextSize(1);
                setCursor(90, 300);
                WriteString("CONTROLLER DISCONNECTED");
                isWarningDisplayed = 1;
                setTextColor(ILI9340_BLACK);
            } else if(isWarningDisplayed && isPadConnected){
                setTextSize(1);
                setCursor(90, 300);
                EraseString("CONTROLLER DISCONNECTED", ILI9340_WHITE);
                isWarningDisplayed = 0;
            }
            // Handle leaving screen test
            if(current_state != STATE_SYS_SCRN_TEST){
                // erase sound test graphical elements
                RemoveArrow(&mainArrow);
                setTextSize(2);
                setCursor(0, 100);
                EraseString("  A: Change color\n\n  B: Change size\n\n  Hold B: Exit", ILI9340_WHITE);
            }
            
        }
        
        
        
        
        while(current_state == STATE_SYS_SND_TEST){                             // STATE_SYS_SND_TEST
            if(newState){
                //draw sound test screen
                tmp = 72;           // used for frequency control
                fillRect(50, 200, 90, 30, ILI9340_BLACK);
                fillRect(80, 170, 30, 90, ILI9340_BLACK);
                fillRect(210, 20, 5, 280, ILI9340_GREEN);
                fillRect(204, 20, 17, 5, ILI9340_GREEN);
                fillRect(204, 300, 17, 5, ILI9340_GREEN);
                setTextColor(ILI9340_BLACK);
                setTextSize(2);
                setCursor(5, 70);
                WriteString("Use UP and DOWN");
                setCursor(5, 90);
                WriteString("to control the");
                setCursor(5, 110);
                WriteString("frequency!");
                setCursor(5, 300);
                WriteString("Press B to exit");
                ArrowDefine(&mainArrow, 185, 130, ILI9340_RED, ILI9340_WHITE, 4);
                isWarningDisplayed = 0;
                newState = 0;
            }
            // lower frequency
            if(button_Down && (tmp < 158)){
                MoveArrow(&mainArrow, 0, 5);
                tmp += 3;
            }
            // raise frequency
            else if(button_Up && (tmp > 3)){
                MoveArrow(&mainArrow, 0, -5);
                tmp -= 3;
            }
            // exit sound test
            else if(button_B){
                current_state = STATE_SYS_TEST_MENU;
                newState = 1;
            }
            // play sound
            Send_SPI_Sound(sineWave128, 128, tmp, 15);
            // Display controller disconnected status message on bottom right
            if(!isPadConnected && !isWarningDisplayed){
                setTextColor(ILI9340_RED);
                setTextSize(1);
                setCursor(90, 300);
                WriteString("CONTROLLER DISCONNECTED");
                isWarningDisplayed = 1;
                setTextColor(ILI9340_BLACK);
            } else if(isWarningDisplayed && isPadConnected){
                setTextSize(1);
                setCursor(90, 300);
                EraseString("CONTROLLER DISCONNECTED", ILI9340_WHITE);
                isWarningDisplayed = 0;
            }
            // Handle leaving sound test
            if(current_state != STATE_SYS_SND_TEST){
                // erase sound test graphical elements
                RemoveArrow(&mainArrow);
                fillRect(50, 200, 90, 30, ILI9340_WHITE);
                fillRect(80, 170, 30, 90, ILI9340_WHITE);
                fillRect(210, 20, 5, 280, ILI9340_WHITE);
                fillRect(204, 20, 17, 5, ILI9340_WHITE);
                fillRect(204, 300, 17, 5, ILI9340_WHITE);
                setTextSize(2);
                setCursor(5, 70);
                EraseString("Use UP and DOWN", ILI9340_WHITE);
                setCursor(5, 90);
                EraseString("to control the", ILI9340_WHITE);
                setCursor(5, 110);
                EraseString("frequency!", ILI9340_WHITE);
                setCursor(5, 300);
                EraseString("Press B to exit", ILI9340_WHITE);
            }
        }
        
        
        
        
        while(current_state == STATE_SYS_GYRO_TEST){                            // STATE_SYS_GYRO_TEST
            if(newState){
                // Draw gyro test screen
                setTextSize(3);
                setTextColor(ILI9340_BLACK);
                setCursor(10, 20);
                WriteString("Ax:");
                setCursor(10, 60);
                WriteString("Ay:");
                setCursor(10, 100);
                WriteString("Az:");
                fillRect(0, 130, 240, 5, ILI9340_BLACK);
                setCursor(10, 140);
                WriteString("Gx:");
                setCursor(10, 180);
                WriteString("Gy:");
                setCursor(10, 220);
                WriteString("Gz:");
                fillRect(0, 250, 240, 5, ILI9340_BLACK);
                setCursor(10, 260);
                WriteString("T:");
                // Display initial readings
                sprintf(bufferAx, "%d", Ax);
                setCursor(70, 20);
                WriteString(bufferAx);
                sprintf(bufferAy, "%d", Ay);
                setCursor(70, 60);
                WriteString(bufferAy);
                sprintf(bufferAz, "%d", Az);
                setCursor(70, 100);
                WriteString(bufferAz);
                sprintf(bufferGx, "%d", Gx);
                setCursor(70, 140);
                WriteString(bufferGx);
                sprintf(bufferGy, "%d", Gy);
                setCursor(70, 180);
                WriteString(bufferGy);
                sprintf(bufferGz, "%d", Gz);
                setCursor(70, 220);
                WriteString(bufferGz);
                sprintf(bufferT, "%d", T);
                setCursor(50, 260);
                WriteString(bufferT);
                // Display Gyro enable status
                if(USE_GYRO){
                    setTextColor(ILI9340_RED);
                    setTextSize(1);
                    setCursor(155, 280);
                    WriteString("GYRO ENABLED");
                }
                else{
                    setTextColor(ILI9340_RED);
                    setTextSize(1);
                    setCursor(150, 280);
                    WriteString("GYRO DISABLED");
                }
                tmp9 = 0;
                newState = 0;
            }
            
            tmp9++; // refresh delay
            //update readings
            if(tmp9 == 1){
                setTextSize(3);
                setTextColor(ILI9340_BLACK);
                setCursor(70, 20);
                EraseString(bufferAx, ILI9340_WHITE);
                sprintf(bufferAx, "%d", Ax);
                setCursor(70, 20);
                WriteString(bufferAx);
                setCursor(70, 60);
                EraseString(bufferAy, ILI9340_WHITE);
                sprintf(bufferAy, "%d", Ay);
                setCursor(70, 60);
                WriteString(bufferAy);
                setCursor(70, 100);
                EraseString(bufferAz, ILI9340_WHITE);
                sprintf(bufferAz, "%d", Az);
                setCursor(70, 100);
                WriteString(bufferAz);
                setCursor(70, 140);
                EraseString(bufferGx, ILI9340_WHITE);
                sprintf(bufferGx, "%d", Gx);
                setCursor(70, 140);
                WriteString(bufferGx);
                setCursor(70, 180);
                EraseString(bufferGy, ILI9340_WHITE);
                sprintf(bufferGy, "%d", Gy);
                setCursor(70, 180);
                WriteString(bufferGy);
                setCursor(70, 220);
                EraseString(bufferGz, ILI9340_WHITE);
                sprintf(bufferGz, "%d", Gz);
                setCursor(70, 220);
                WriteString(bufferGz);
                setCursor(50, 260);
                EraseString(bufferT, ILI9340_WHITE);
                sprintf(bufferT, "%d", T);
                setCursor(50, 260);
                WriteString(bufferT);
                tmp9 = 0;
            }
            
            // exit sound test
            if(button_B){
                current_state = STATE_SYS_TEST_MENU;
                newState = 1;
            }
            
            // Display controller disconnected status message on bottom right
            if(!isPadConnected && !isWarningDisplayed){
                setTextColor(ILI9340_RED);
                setTextSize(1);
                setCursor(90, 300);
                WriteString("CONTROLLER DISCONNECTED");
                isWarningDisplayed = 1;
                setTextColor(ILI9340_BLACK);
            } else if(isWarningDisplayed && isPadConnected){
                setTextSize(1);
                setCursor(90, 300);
                EraseString("CONTROLLER DISCONNECTED", ILI9340_WHITE);
                isWarningDisplayed = 0;
            }
            
            // Handle leaving gyro test
            if(current_state != STATE_SYS_GYRO_TEST){
                // erase gyro test graphical elements
                if(USE_GYRO){
                    setTextSize(1);
                    setCursor(155, 280);
                    EraseString("GYRO ENABLED", ILI9340_WHITE);
                }
                else{
                    setTextSize(1);
                    setCursor(150, 280);
                    EraseString("GYRO DISABLED", ILI9340_WHITE);
                }
                setTextSize(3);
                setCursor(70, 20);
                EraseString(bufferAx, ILI9340_WHITE);
                setCursor(70, 60);
                EraseString(bufferAy, ILI9340_WHITE);
                setCursor(70, 100);
                EraseString(bufferAz, ILI9340_WHITE);
                setCursor(70, 140);
                EraseString(bufferGx, ILI9340_WHITE);
                setCursor(70, 180);
                EraseString(bufferGy, ILI9340_WHITE);
                setCursor(70, 220);
                EraseString(bufferGz, ILI9340_WHITE);
                setCursor(50, 260);
                EraseString(bufferT, ILI9340_WHITE);
                setCursor(10, 20);
                EraseString("Ax:", ILI9340_WHITE);
                setCursor(10, 60);
                EraseString("Ay:", ILI9340_WHITE);
                setCursor(10, 100);
                EraseString("Az:", ILI9340_WHITE);
                fillRect(0, 130, 240, 5, ILI9340_WHITE);
                setCursor(10, 140);
                EraseString("Gx:", ILI9340_WHITE);
                setCursor(10, 180);
                EraseString("Gy:", ILI9340_WHITE);
                setCursor(10, 220);
                EraseString("Gz:", ILI9340_WHITE);
                fillRect(0, 250, 240, 5, ILI9340_WHITE);
                setCursor(10, 260);
                EraseString("T:", ILI9340_WHITE);
            }
            
        }
        
        
        
        
        while(current_state == STATE_SYS_ERROR){                                // STATE_SYS_ERROR
            if(newState){
                // display error message
                fillRect(12, 135, 211, 58, ILI9340_BLUE);
                Send_SPI_Sound(sineWave128, 128, 95, 10);
                setTextSize(2);
                setTextColor(ILI9340_WHITE);
                setCursor(17, 140);
                WriteString("Unexpected error.");
                setCursor(17, 172);
                WriteString("Press any button.");
                Send_SPI_Sound(sineWave128, 128, 95, 70);
                newState = 0;
            }
            // restart system at any button press
            if(buttons){ 
                current_state = STATE_SYSTEM_BOOT;
                newState = 1;
                fillScreen(ILI9340_WHITE); // blank screen
            }
            
        }
    
    
        while(current_state == STATE_SUPER_MARIO){                              // STATE_SUPER_MARIO
            if(newState){
                // Draw sky / background
                fillRect(0, 0, 240, 288, ILI9340_CYAN);
                setTextColor(ILI9340_WHITE);
                setTextSize(2);
                setCursor(10, 10);
                WriteString("SCORE");
                setTextSize(2);
                setCursor(15, 30);
                WriteString("0");
                #define MARIO_FLOOR 288 // Super Mario Test floor level
                // Draw floor tiles
                for(tmp = 0; tmp < 240; tmp+=32)
                    Draw_Sprite(sprite_Block_Ground, tmp, MARIO_FLOOR, 2, false);
                // Draw Mario
                New_Sprite(&player, 80, 100, 2, sprite_Mario_Still, false, ILI9340_CYAN);
                // Init variables used in this state
                tmp = 0; // used here for isRunning bool
                tmp_x = 0; // used for player x velocity
                tmp_y = 1; // used for player y velocity
                tmp2 = 0; // tmp2 here used for player animation frame
                tmp3 = 0; // tmp3 here used for timer for animations
                tmp4 = 1; // used here for isJumping bool
                tmp5 = 0; // used here to keep track of score
                tmp8 = 100000; // used here for countdown timer for bullet enemy to reappear
                tmp9 = 0; // used here for "DEMO" flash frequency counter
                // The usual cleanup
                isWarningDisplayed = 0;
                newState = 0;
                // Play first part of SMB theme
                Send_SPI_Sound(sineWave128, 128, 35, 25);
                Send_SPI_Sound(silence, 2, 100, 150);
                Send_SPI_Sound(sineWave128, 128, 35, 30);
                Send_SPI_Sound(silence, 2, 250, 250);
                Send_SPI_Sound(sineWave128, 128, 35, 70);
                Send_SPI_Sound(silence, 2, 250, 250);
                Send_SPI_Sound(sineWave128, 128, 50, 20);
                Send_SPI_Sound(silence, 2, 250, 250);
                Send_SPI_Sound(sineWave128, 128, 35, 70);
                Send_SPI_Sound(silence, 2, 250, 250);
                Send_SPI_Sound(sineWave128, 128, 25, 160);
                Send_SPI_Sound(silence, 2, 500, 500);
                Send_SPI_Sound(sineWave128, 128, 60, 100);
            }
            
            // Check pause button
            if(button_Start){
                setTextSize(3);
                setCursor(75, 10);
                EraseString("DEMO", ILI9340_CYAN);
                setTextColor(ILI9340_WHITE);
                setCursor(70, 130);
                WriteString("PAUSE");
                Send_SPI_Sound(sineWave128, 128, 5, 30);
                Send_SPI_Sound(sineWave128, 128, 10, 30);
                Send_SPI_Sound(sineWave128, 128, 5, 30);
                Send_SPI_Sound(sineWave128, 128, 10, 30);
                // remain paused until START button pressed again
                while(!button_Start){}
                setCursor(70, 130);
                EraseString("PAUSE", ILI9340_CYAN);
                Send_SPI_Sound(sineWave128, 128, 5, 30);
                Send_SPI_Sound(sineWave128, 128, 10, 30);
                Send_SPI_Sound(sineWave128, 128, 5, 30);
                Send_SPI_Sound(sineWave128, 128, 10, 30);
            }
            
            // Player movement logic
            // running?
            if(button_B){
                tmp = 1;
                rand(); // user-input for higher randomness
            }
            else{
                tmp = 0;
            }
            // move sprite left
            if(button_Left){
                if(tmp) // when running
                    tmp_x = -9;
                else{ // else walking
                    tmp_x = -5;
                    rand(); // user-input for higher randomness
                }
            }
            // move sprite right
            else if(button_Right){
                if(tmp) // when running
                    tmp_x = 9;
                else // else walking
                    tmp_x = 5;
            }
            // not moving on x-axis
            else{
                tmp_x = 0;
            }
            // Jump (if not already jumping)
            if(button_A && !tmp4){
                tmp_y = -12;
                tmp4 = 1;
                Send_SPI_Sound(sineWave128, 128, 35, 4);
                Send_SPI_Sound(sineWave128, 128, 15, 6);
                rand(); // user-input for higher randomness
            }
            
            // Gravity (if jumping)
            if(tmp4){
                tmp_y++;
            }
            
            // Actually move the player (if moving)
            if((tmp_x != 0) || (tmp_y != 0)){ // if sprite needs to be updated
                if(tmp4){ // if jumping
                    if((player.curr_y + tmp_y + (16*player.size)) > MARIO_FLOOR){ // if player will go into floor
                        tmp_y = 0; // no y velocity
                        tmp4 = 0; // not jumping
                        if(tmp_x > 0)
                            Move_Sprite(&player, tmp_x, (MARIO_FLOOR - player.curr_y - (16*player.size)), anim_Mario_Walking[tmp2], 1);
                        else if (tmp_x < 0)
                            Move_Sprite(&player, tmp_x, (MARIO_FLOOR - player.curr_y - (16*player.size)), anim_Mario_Walking[tmp2], 2);
                        else
                            Move_Sprite(&player, tmp_x, (MARIO_FLOOR - player.curr_y - (16*player.size)), sprite_Mario_Still, 0);
                    }
                    else{ // still jumping
                        if(tmp_x > 0)
                            Move_Sprite(&player, tmp_x, tmp_y, sprite_Mario_Jump, 1);
                        else if (tmp_x < 0)
                            Move_Sprite(&player, tmp_x, tmp_y, sprite_Mario_Jump, 2);
                        else
                            Move_Sprite(&player, tmp_x, tmp_y, sprite_Mario_Jump, 0);
                    }
                }
                else{ // else walking or running
                    if(tmp_x > 0)
                        Move_Sprite(&player, tmp_x, tmp_y, anim_Mario_Walking[tmp2], 1);
                    else if (tmp_x < 0)
                        Move_Sprite(&player, tmp_x, tmp_y, anim_Mario_Walking[tmp2], 2);
                    else
                        Move_Sprite(&player, tmp_x, tmp_y, sprite_Mario_Still, 0);
                }
            }
            
            // handle collisions with screen border
            if(player.curr_x > SCREEN_WIDTH-(16*player.size)){
                Sprite_Change_Loc(&player, SCREEN_WIDTH-(16*player.size), -1, NULL);
            }
            else if(player.curr_x < 0){
                Sprite_Change_Loc(&player, 0, -1, NULL);
            }
            if(player.curr_y > SCREEN_HEIGHT-(16*player.size)){
                Sprite_Change_Loc(&player, -1, SCREEN_HEIGHT-(16*player.size), NULL);
            }
            else if(player.curr_y < 0){
                Sprite_Change_Loc(&player, -1, 0, NULL);
            }
            
            // Correct player sprite if in/under floor
            if(player.curr_y > MARIO_FLOOR){
                Sprite_Change_Loc(&player, -1, MARIO_FLOOR, sprite_Mario_Still);
                tmp_y = 0; // no y velocity
                tmp4 = 0; // no longer jumping
            }
            
            // Make sure player sprite graphic is correct when standing still
            if((tmp_x == 0) && (!tmp4) && (player.graphic != sprite_Mario_Still))
                Sprite_Change_Loc(&player, -1, -1, sprite_Mario_Still);
            
            // Update Mario walking animation array
            tmp2++;
            if(tmp2>=4){tmp2=0;}
            
            // Bullet Bill logic
            tmp8--;
            if((tmp8 == 0) && (!bullet.draw)){ // spawn in Bullet Bill
                enemy_type = (rand() % (3 - 0 + 1));        // random enemy type
                enemy_height = (rand() % (40 - 0 + 1));     // random enemy y offset
                switch(enemy_type){
                    case 0: // size 2 bullet bill from right
                        Send_SPI_Sound(sineWave128, 128, 50, 5);
                        Send_SPI_Sound(sineWave128, 128, 70, 5);
                        New_Sprite(&bullet, SCREEN_WIDTH, MARIO_FLOOR-32-enemy_height, 2, sprite_Enemy_Bullet, false, ILI9340_CYAN);
                        break;
                        
                    case 1: // size 1 bullet bill from right
                        Send_SPI_Sound(sineWave128, 128, 20, 5);
                        Send_SPI_Sound(sineWave128, 128, 35, 5);
                        New_Sprite(&bullet, SCREEN_WIDTH, MARIO_FLOOR-16-enemy_height, 1, sprite_Enemy_Bullet, false, ILI9340_CYAN);
                        break;
                        
                    case 2: // size 2 bullet from left
                        Send_SPI_Sound(sineWave128, 128, 50, 5);
                        Send_SPI_Sound(sineWave128, 128, 70, 5);
                        New_Sprite(&bullet, -32, MARIO_FLOOR-32-enemy_height, 2, sprite_Enemy_Bullet, true, ILI9340_CYAN);
                        break;
                        
                    case 3: // size 1 bullet from left
                        Send_SPI_Sound(sineWave128, 128, 20, 5);
                        Send_SPI_Sound(sineWave128, 128, 35, 5);
                        New_Sprite(&bullet, -32, MARIO_FLOOR-16-enemy_height, 1, sprite_Enemy_Bullet, true, ILI9340_CYAN);
                        break;
                        
                }
                tmp8 = (rand() % (10000 - 50000 + 1)); // random respawn time
            }
            if(bullet.draw){ // if currently on screen (exists/draw)
                switch(enemy_type){
                    case 0: // size 2 bullet bill from right
                        Move_Sprite(&bullet, -12, 0, sprite_Enemy_Bullet, 0);
                        if((bullet.curr_x < -32) || (bullet.curr_y > SCREEN_WIDTH + 32)) // remove when off screen
                            bullet.draw = false;
                        break;
                        
                    case 1: // size 1 bullet bill from right
                        Move_Sprite(&bullet, -8, 0, sprite_Enemy_Bullet, 0);
                        if((bullet.curr_x < - 32) || (bullet.curr_y > SCREEN_WIDTH + 32)) // remove when off screen
                            bullet.draw = false;
                        break;
                        
                    case 2: // size 2 bullet from left
                        Move_Sprite(&bullet, 12, 0, sprite_Enemy_Bullet, 0);
                        if((bullet.curr_x > SCREEN_WIDTH + 32) || (bullet.curr_y > SCREEN_WIDTH + 32)) // remove when off screen
                            bullet.draw = false;
                        break;
                        
                    case 3: // size 1 bullet from left
                        Move_Sprite(&bullet, 8, 0, sprite_Enemy_Bullet, 0);
                        if((bullet.curr_x > SCREEN_WIDTH + 32) || (bullet.curr_y > SCREEN_WIDTH + 32)) // remove when off screen
                            bullet.draw = false;
                        break;
                }
                if(!bullet.draw){ // if bullet went off screen, Update SCORE display
                    setTextSize(2);
                    setCursor(15, 30);
                    EraseString(intString[tmp5], ILI9340_CYAN);
                    tmp5++; // score++
                    setCursor(15, 30);
                    WriteString(intString[tmp5]);
                    if(tmp5 == 26){ // 26 points to win
                        setTextSize(3);
                        setCursor(45, 130);
                        WriteString("YOU WIN!");
                        setTextSize(2);
                        setCursor(70, 170);
                        WriteString("SCORE: ");
                        WriteString(intString[tmp5]);
                        while(!buttons){} // wait for button press
                        newState = 1; // restart game
                    }
                }
            }
            
            // Check player collision with enemy
            if((player.curr_x+22 > bullet.curr_x && player.curr_x < bullet.curr_x+((bullet.size*16)-4)) && (player.curr_y < bullet.curr_y+((bullet.size*16)-2) && player.curr_y+32 > bullet.curr_y+4) && (bullet.draw)){
                // Player dies, game over
                Send_SPI_Sound(sineWave128, 128, 30, 5);
                Sprite_Change_Loc(&player, -1, -1, sprite_Mario_Death);
                Send_SPI_Sound(sineWave128, 128, 35, 50);
                Send_SPI_Sound(silence, 2, 100, 150);
                Send_SPI_Sound(sineWave128, 128, 15, 120);
                Send_SPI_Sound(silence, 2, 100, 150);
                Send_SPI_Sound(sineWave128, 128, 20, 50);
                Send_SPI_Sound(silence, 2, 100, 50);
                Send_SPI_Sound(sineWave128, 128, 15, 110);
                Send_SPI_Sound(silence, 2, 100, 150);
                Send_SPI_Sound(sineWave128, 128, 20, 95);
                Send_SPI_Sound(silence, 2, 100, 150);
                Send_SPI_Sound(sineWave128, 128, 35, 90);
                Send_SPI_Sound(silence, 2, 100, 150);
                Send_SPI_Sound(sineWave128, 128, 65, 150);
                setTextSize(2);
                setCursor(10, 10);
                EraseString("SCORE", ILI9340_CYAN);
                setTextSize(3);
                setCursor(75, 10);
                EraseString("DEMO", ILI9340_CYAN);
                setTextSize(2);
                setCursor(15, 30);
                EraseString(intString[tmp5], ILI9340_CYAN);
                setTextSize(3);
                setCursor(40, 130);
                WriteString("GAME OVER");
                setTextSize(2);
                setCursor(70, 170);
                WriteString("SCORE: ");
                WriteString(intString[tmp5]);
                while(!buttons){} // wait for button press
                bullet.draw = false;
                newState = 1; // restart game
            }

            // Flash "DEMO"
            tmp9++;
            if(tmp9 == 80000){
                setTextColor(ILI9340_WHITE);
                setTextSize(3);
                setCursor(75, 10);
                WriteString("DEMO");
            }
            else if(tmp9 == 150000){
                setTextSize(3);
                setCursor(75, 10);
                EraseString("DEMO", ILI9340_CYAN);
                tmp9 = 0;
            }
            
        }
        
    }
        
    return (EXIT_SUCCESS);
}


// controller read interrupt (T1 & T2 unused)
void __attribute__((interrupt(no_auto_psv))) _T1Interrupt(void){
    IFS0bits.T1IF= 0;       // clear Timer1 interrupt flag
}

void __attribute__((interrupt(no_auto_psv))) _T2Interrupt(void){
    IFS0bits.T2IF = 0;      // Clear interrupt flag
}

void __attribute__((interrupt(no_auto_psv))) _T3Interrupt(void){
    IFS0bits.T3IF = 0;      // Clear interrupt flag
    if(!ext_pad_check_cooldown_timer){
        Controller_Read();      // Read ext controller
        if(!isPadConnected){
            OBC_Read();
            ext_pad_check_cooldown_timer = PADCHECKCOOLDOWN; // set controller reconnect check cooldown timer to reduce OBC lag
        }
    }
    else{ // when ext pad not detected
        OBC_Read();
        ext_pad_check_cooldown_timer--;
    }
}