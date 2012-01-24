// This program allows control of P1.0 via the backchannel serial
// interface on the Launchpad. In order to achieve this, an
// interrupt-based half-duplex software UART implementation is
// included.
//
// Connect to the Launchpad at 9600 8N1 and send 'I' to set P1.0 high
// or 'O' to set P1.0 low.

#include <msp430.h>
#include <msp430g2231.h>
#include <stdbool.h>

#define  RELAY    BIT0  // control relay on P1.0
#define  SER_TXD  BIT1  // transmit serial on P1.1
#define  SER_RXD  BIT2  // receive serial on P1.2

#define  BIT_TIME       104  // clocks per bit for 9600 baud and 1MHz clock
#define  HALF_BIT_TIME  52   // clocks per half bit

unsigned char bit_count;
unsigned int tx_byte;
unsigned int rx_byte;

bool is_receiving;
bool has_received;

void transmit(void);

int main(void) {
    WDTCTL = WDTPW + WDTHOLD; // Stop the watchdog timer

    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

    P1SEL |= SER_TXD;
    P1DIR |= SER_TXD + RELAY;

    P1OUT |= RELAY;

    P1IES |= SER_RXD;  // enable RXD hi/lo edge interrupt
    P1IFG &= ~SER_RXD; // clear RXD flag before enabling interrupt
    P1IE |= SER_RXD;   // enable interrupt

    is_receiving = false;
    has_received = false;

    __bis_SR_register(GIE);

    while (1) {
        if (has_received) {
            has_received = false;
            if (rx_byte == 'I') {
                // 0x1 means pull port high (turn off relay)
                P1OUT |= RELAY;
            }
            else if (rx_byte == 'O') {
                // 0x2 means pull port low (turn on relay)
                P1OUT &= ~RELAY;
            }

            // echo the received byte back to the sender
            tx_byte = rx_byte;
            transmit();
        }

        if (~has_received)
            __bis_SR_register(CPUOFF + GIE);
    }
}

// place a character in tx_byte and call this to transmit it.
void transmit(void) {
    while (is_receiving);
    CCTL0 = OUT;
    TACTL = TASSEL_2 + MC_2;

    bit_count = 0xa;
    CCR0 = TAR;

    CCR0 += BIT_TIME;
    tx_byte |= 0x100;
    tx_byte = tx_byte << 1;

    CCTL0 = CCIS0 + OUTMOD0 + CCIE;
    while (CCTL0 & CCIE);
}

__attribute__((interrupt(PORT1_VECTOR)))
void PORT1_ISR(void) {
    is_receiving = true;

    P1IE &= ~SER_RXD;
    P1IFG &= ~SER_RXD;

    TACTL = TASSEL_2 + MC_2;
    CCR0 = TAR;
    CCR0 += HALF_BIT_TIME;
    CCTL0 = OUTMOD1 + CCIE;

    rx_byte = 0;
    bit_count = 0x9;
}

__attribute__((interrupt(TIMERA0_VECTOR)))
void TIMERA0_ISR(void) {
    if (!is_receiving) {
        CCR0 += BIT_TIME;
        if (bit_count == 0) {
            TACTL = TASSEL_2;
            CCTL0 &= ~CCIE;
        }
        else {
            CCTL0 |= OUTMOD2;
            if (tx_byte & 0x01)
                CCTL0 &= ~OUTMOD2;
            tx_byte = tx_byte >> 1;
            --bit_count;
        }
    }
    else {
        CCR0 += BIT_TIME;
        if (bit_count == 0) {
            TACTL = TASSEL_2;
            CCTL0 &= ~CCIE;

            is_receiving = false;

            P1IFG &= ~SER_RXD;
            P1IE |= SER_RXD;

            if ((rx_byte & 0x201) == 0x200) {
                rx_byte = rx_byte >> 1;
                rx_byte &= 0xff;
                has_received = true;
            }

            __bic_SR_register_on_exit(CPUOFF);
        }
        else {
            if ((P1IN & SER_RXD) == SER_RXD)
                rx_byte |= 0x400;
            rx_byte = rx_byte >> 1;
            --bit_count;
        }
    }
}

