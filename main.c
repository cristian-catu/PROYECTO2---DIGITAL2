/*
 * File:   main.c
 * Author: Pablo
 * Ejemplo de uso de I2C Master
 * Created on 17 de febrero de 2020, 10:32 AM
 */
//*****************************************************************************
// Palabra de configuración
//*****************************************************************************
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//*****************************************************************************
// Definición e importación de librerías
//*****************************************************************************
#include <stdint.h>
#include <pic16f887.h>
#include <xc.h>
#include <stdio.h>
//*****************************************************************************
// Definición de variables
//*****************************************************************************
#define _XTAL_FREQ 1000000

//*****************************************************************************
// Definición de funciones para que se puedan colocar después del main de lo 
// contrario hay que colocarlos todas las funciones antes del main
//*****************************************************************************
void setup(void);
uint8_t z;
uint8_t dato;

uint8_t hours = 12;                             // Variable de 8 bits para las horas.
uint8_t minutes = 59;                           // Variable de 8 bits para los minutos.
uint8_t seconds = 45;                           // Variable de 8 bits para los segundos.
uint8_t POT1 = 0;
uint8_t POT2 = 0;
uint8_t canal_ADC = 0;
uint8_t VALOR_RECIBIDO = 0;
uint8_t VALOR_A_ENVIAR = 0;
uint8_t entrada = 0;
uint8_t entrada2 = 0;
uint8_t bandera = 0;
uint8_t PUSH = 0;
uint8_t PUSH2 = 0;
uint8_t bandera2 = 0;
uint8_t bandera3 = 0;
uint8_t POTENCIOMETRO2 = 25;
uint8_t cambiar = 0;
uint8_t temperatura = 0;
uint8_t adelante = 0;
uint8_t estado = 0;
uint8_t direccion = 0;
uint8_t mover = 0;
uint8_t horas = 1;
uint8_t PUERTO1 = 0;

char s[];
unsigned short VOLTAJE_0 = 0;
uint8_t init_POT_0 = 0;
uint8_t dec_POT_0 = 0;
uint8_t VAL_POT_0 = 0;

//*****************************************************************************
// Main
//*****************************************************************************

void __interrupt() isr(void){
    if(INTCONbits.RBIF){ //Interrupción del puerto B
        if (PUERTO1 == 0){
            if (!PORTBbits.RB0){
                TXREG = 65;
                PORTAbits.RA0 = 1;
                PUERTO1 = 1;
            }

            else if (!PORTBbits.RB1){
                TXREG = 67;
                PORTAbits.RA1 = 1;
                PUERTO1 = 2;
            }
            else if (!PORTBbits.RB2){
                TXREG = 69;
                PORTAbits.RA2 = 1;
                PUERTO1 = 3; 
            }
        }
        else if(PORTBbits.RB2 && PORTBbits.RB1 && PORTBbits.RB0) {
            if (PUERTO1 == 1){
                TXREG = 66;
                PORTAbits.RA0 = 0;
                PUERTO1 = 0;
            }
            else if (PUERTO1 == 2){
                TXREG = 68;
                PORTAbits.RA1 = 0;
                PUERTO1 = 0;
            }
            else if (PUERTO1 == 3){
                TXREG = 70;
                PORTAbits.RA2 = 0;
                PUERTO1 = 0;
            }
        }
        
        
        INTCONbits.RBIF = 0;
        
    }
    return;
}

void main(void) {
    setup();
    while(1){     
    }
    return;
}
//*****************************************************************************
// Función de Inicialización
//*****************************************************************************
void setup(void){
    ANSEL = 0;
    ANSELH = 0;
    TRISB = 0b00000111;
    TRISC = 0b10000000;
    TRISA = 0;
    PORTA = 0;
    TRISD = 0;
    PORTB = 0;
    PORTD = 0;
    //PORTB
    
    OPTION_REGbits.nRBPU = 0; 
    WPUB = 0b00000111; 
    INTCONbits.RBIE = 1;   
    IOCB = 0b00000111;         
    INTCONbits.RBIF = 0;  
    
    OSCCONbits.SCS = 1;
    OSCCONbits.IRCF = 0b100;
    
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    PIE1bits.RCIE = 1;          // Habilitamos Interrupciones de recepción
    // Configuraciones de comunicacion serial
    //SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG=25 <- Valores de tabla 12-5
    TXSTAbits.SYNC = 0;         // Comunicación ascincrona (full-duplex)
    TXSTAbits.BRGH = 1;         // Baud rate de alta velocidad 
    BAUDCTLbits.BRG16 = 1;      // 16-bits para generar el baud rate
    
    SPBRG = 25;
    SPBRGH = 0;                 // Baud rate ~9600, error -> 0.16%
    
    RCSTAbits.SPEN = 1;         // Habilitamos comunicación
    TXSTAbits.TX9 = 0;          // Utilizamos solo 8 bits
    TXSTAbits.TXEN = 1;         // Habilitamos transmisor
    RCSTAbits.CREN = 1;         // Habilitamos receptor
}