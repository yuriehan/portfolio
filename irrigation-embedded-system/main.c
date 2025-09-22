// Code originally from:
// https://github.com/andrei-cb/I2C-Display-and-MSP430/tree/master
// Then modified by Joe Young

#include "lcd.h"
#include "msp430g2553.h"
#include "stdio.h"

// Define soil moisture thresholds based on your calibration values
#define SOIL_WET_THRESHOLD 500
#define SOIL_DRY_THRESHOLD 750

// Function prototypes
void ADC_Init();
int ReadSensor();

int main()
{
    WDTCTL = WDTPW + WDTHOLD; // Stop watchdog

    // Set P1.2 as ADC input
    P1SEL |= BIT3;
    P1SEL2 |= BIT3;

    //Set P2.3 as output
    P2DIR |= BIT3;

    // Enable global interrupts
    __enable_interrupt();

    LcdInit(); // initialize LCD display
    ADC_Init(); // Initialize ADC

    char text[20]; // buffer

    while (1)
    {
        int moisture = ReadSensor();

        // Clear the LCD screen
        LcdWriteCommand(LCD_CLEAR, 1);

        // Display moisture level on LCD
        LcdSetPosition(1, 1); // Set cursor to second line
        sprintf(text, "Moisture: %d    ", moisture);
        LcdWriteString(text);
        LcdSetPosition(2, 1); // Set cursor to second line

        // Determine status of soil moisture
        if (moisture < SOIL_WET_THRESHOLD)
        {
            LcdWriteString("Soil too wet");
            //Set P2.3 to on
            P2OUT |= BIT3;
        }
        else if (moisture >= SOIL_WET_THRESHOLD && moisture < SOIL_DRY_THRESHOLD)
        {
            LcdWriteString("Moisture OK");
            //Set P2.3 to on
            P2OUT |= BIT3;
        }
        else
        {
            LcdWriteString(" Water needed  ");
            // Set P2.3 to OFF state
            P2OUT &= ~BIT3;
        }
        __delay_cycles(4000000); // Delay for testing (adjust as needed)
    }

    return 0;
}

// Initialize ADC
void ADC_Init()
{
    ADC10CTL1 = INCH_3; // Select channel 2 (P1.2)
    ADC10CTL0 = SREF_1 + ADC10SHT_2 + ADC10ON + REFON; // VCC/VSS ref, 64 x ADC10CLKs
}

// Read soil moisture sensor
int ReadSensor()
{
    ADC10CTL0 |= ENC + ADC10SC; // Enable conversion and start conversion
    while (ADC10CTL1 & ADC10BUSY); // Wait until conversion is complete
    return ADC10MEM; // Return result
}

