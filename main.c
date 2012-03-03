// This program allows control of P1.0 and P2.0 via the backchannel serial
// interface on the Launchpad. In order to achieve this, an
// interrupt-based half-duplex software UART implementation is
// included.
//
// Connect to the Launchpad at 9600 8N1 and send 'I' to set P1.0 high
// or 'O' to set P1.0 low, 'J' to set P2.0 high or 'P' to set P2.0 low

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
    WDTCTL = WDTPW + WDTHOLD; // stop the watchdog timer

    BCSCTL1 = CALBC1_1MHZ; // set range
    DCOCTL = CALDCO_1MHZ;  // SMCLK = DCO = 1MHz

    P1SEL |= SER_TXD;
    P1DIR |= SER_TXD + RELAY;
    P2DIR |= RELAY;

    P1OUT |= RELAY;
    P2OUT |= RELAY;

    P1IES |= SER_RXD;  // enable RXD hi/lo edge interrupt
    P1IFG &= ~SER_RXD; // clear RXD flag before enabling interrupt
    P1IE |= SER_RXD;   // enable RXD interrupt

    is_receiving = false;
    has_received = false;

    __bis_SR_register(GIE); // globally enable interrupts

    while (1) {
        if (has_received) {
            has_received = false;
            if (rx_byte == 'I') {
                P1OUT |= RELAY; // pull relay 1 high
            }
            else if (rx_byte == 'J') {
                P2OUT |= RELAY; // pull relay 2 high
            }
            else if (rx_byte == 'O') {
                P1OUT &= ~RELAY; // pull relay 1 low
            }
            else if (rx_byte == 'P') {
                P2OUT &= ~RELAY; // pull relay 2 low
            }

            // echo the received byte back to the sender
            tx_byte = rx_byte;
            transmit();
        }

        // if there is not another value received, turn off the CPU until an interrupt occurs
        // (power saving)
        if (~has_received)
            __bis_SR_register(CPUOFF + GIE);
    }
}

// place a character in tx_byte and call this to transmit it.
void transmit(void) {
    while (is_receiving); // half-duplex: wait until RX is complete
    CCTL0 = OUT; // TXD idle as mark
    TACTL = TASSEL_2 + MC_2; // SMCLK continuous

    bit_count = 0xa; // load bit counter, 8 bits + start + stop
    CCR0 = TAR; // initialise compare reg

    CCR0 += BIT_TIME; // set time until first bit
    tx_byte |= 0x100; // add stop bit (logical 1) to tx_byte
    tx_byte = tx_byte << 1; // add start bit (logical 0)

    CCTL0 = CCIS0 + OUTMOD0 + CCIE; // set signal, initial value, enable interrupts
    while (CCTL0 & CCIE); // wait for previous TX completion
}

__attribute__((interrupt(PORT1_VECTOR)))
void PORT1_ISR(void) {
    is_receiving = true;

    P1IE &= ~SER_RXD; // disable RXD interrupt
    P1IFG &= ~SER_RXD; // clear interrupt flag

    TACTL = TASSEL_2 + MC_2; // SMCLK continuous mode
    CCR0 = TAR; // initialise compare register
    CCR0 += HALF_BIT_TIME; // set time until first bit
    CCTL0 = OUTMOD1 + CCIE; // disable TX and enable interrupts

    rx_byte = 0; // initialise rx_byte
    bit_count = 0x9; // load bit counter, 8 bits + start
}

__attribute__((interrupt(TIMERA0_VECTOR)))
void TIMERA0_ISR(void) {
    if (!is_receiving) {
        CCR0 += BIT_TIME; // add offset to CCR0
        if (bit_count == 0) { // if all bits sent
            TACTL = TASSEL_2; // SMCLK timer off (power saving)
            CCTL0 &= ~CCIE; // disable interrupt
        }
        else {
            CCTL0 |= OUTMOD2; // set TX bit to 0
            if (tx_byte & 0x01)
                CCTL0 &= ~OUTMOD2; // if it should be 1, set it so
            tx_byte = tx_byte >> 1;
            --bit_count;
        }
    }
    else {
        CCR0 += BIT_TIME; // add offset to CCR0
        if (bit_count == 0) {
            TACTL = TASSEL_2; // SMCLK timer off (power saving)
            CCTL0 &= ~CCIE; // disable interrupt

            is_receiving = false;

            P1IFG &= ~SER_RXD; // clear RXD interrupt flag
            P1IE |= SER_RXD; // enable RXD interrupt

            if ((rx_byte & 0x201) == 0x200) { // validate start and stop bits
                rx_byte = rx_byte >> 1; // remove start bit
                rx_byte &= 0xff; // remove stop bit
                has_received = true;
            }

            __bic_SR_register_on_exit(CPUOFF); // enable CPU so that main while loop continues
        }
        else {
            if ((P1IN & SER_RXD) == SER_RXD) // if bit is set
                rx_byte |= 0x400; // set value in rx_byte
            rx_byte = rx_byte >> 1; // shift bits down
            --bit_count;
        }
    }
}

