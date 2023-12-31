#include "Atmega328PB.h"
#include <util/delay.h>

void blink_LED() {

  static uint8_t counter = 0;       // Create variable counter

  if (counter == 125) {             // 0.5s at 250ms delay
    LATBbits.LATB5 = ~LATBbits.RB5; // Toggle LED  => using bits avr-gcc PORTB ^= _BV(PORTB5);
    counter = 0;                    // Reset counter
  }
  else {
    counter++;                      // Increment counter
  }
}

int main(int argc, char const* argv[])
{

  TRISBbits.RB5 = 1;                // set PB5 output

  for (;;)
  {

    blink_LED();
    _delay_ms(2);

  }

  return 0;
}


/*******************************************************
 * example usart configuration with this library.
 * use a similar methods than xc8 compiler with
 * avr-gcc
 *
 * void USART_Init(unsigned int baud) {
 *    UBRR0 = ((F_CPU / (16UL * baud)) - 1);                                                      // Set baud rate from equations Asynchonous normal mode (UBRRn = (fosc/16BAUD) -1)
 *    UCSR0Bbits.RXEN = 1;                                                                        // Receiver enable
 *    UCSR0Bbits.TXEN = 1;                                                                        // Transmitter enable
 *    // sets number to data bits in a frame the Receiver and Transmitter use. => 8 bits
 *    UCSR0Cbits.UCSZ0 = 1;
 *    UCSR0Cbits.UCSZ1 = 1;
 * }
 *********************************************************/