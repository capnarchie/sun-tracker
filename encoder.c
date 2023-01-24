#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

// Constants
#define ENCODER_A_PIN  PD2  // Pin for encoder 1A
#define ENCODER_B_PIN  PD3  // Pin for encoder 1B
uint16_t angle_per_count = 360 / (1000 * 12); // = 0.432 degrees/count
// Variables
volatile int16_t encoder_count = 0;  // Encoder count

// Interrupt service routine for encoder A
ISR(PCINT0_vect)
{
    // Read encoder A and B
    //todo
    uint8_t encoder_a;
    uint8_t encoder_b;
    // Increment or decrement encoder count based on encoder A and B
    if (encoder_a)
    {
        if (encoder_b)
        {
            encoder_count--;
        }
        else
        {
            encoder_count++;
        }
    }
    else
    {
        if (encoder_b)
        {
            encoder_count++;
        }
        else
        {
            encoder_count--;
        }
    }
}

int main(void)
{
    // Set encoder A and B as inputs
    DDRD &= ~_BV(ENCODER_A_PIN);
    DDRD &= ~_BV(ENCODER_B_PIN);

    // Enable pin change interrupt for encoder A
    PCMSK0 |= _BV(ENCODER_A_PIN);
    PCICR |= _BV(PCIE0);

    // Enable global interrupts
    sei();

    while (1)
    {
        // Print encoder count
        //printf("Encoder count: %d\n", encoder_count);
        uint8_t current_angle = encoder_count * angle_per_count
        // Delay 1 second
        _delay_ms(1000);
    }

    return 0;
}
