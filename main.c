/* vim: set ai et ts=4 sw=4: */

#include <avr/io.h>
#include <util/delay.h>

#ifndef BAUD
#define BAUD 9600
#endif

#include <util/setbaud.h>

#define DS1302_CLK        PD6 // arduino pin 6
#define DS1302_CLK_DDR    DDRD
#define DS1302_CLK_PORT   PORTD

#define DS1302_DIO        PD5 // arduino pin 5
#define DS1302_DIO_DDR    DDRD
#define DS1302_DIO_PORT   PORTD
#define DS1302_DIO_PIN    PIND

#define DS1302_CE         PD4 // arduino pin 4
#define DS1302_CE_DDR     DDRD
#define DS1302_CE_PORT    PORTD

#define DS1302_DELAY_USEC 3

void UART_Init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
    #if USE_2X
    UCSR0A |= (1 << U2X0);
    #else
    UCSR0A &= ~(1 << U2X0);
    #endif

    /* Enable UART transmitter/receiver */
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    /* 8 data bits, 1 stop bit */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_TransmitByte(uint8_t data) {
    /* Wait for empty transmit buffer */
    loop_until_bit_is_set(UCSR0A, UDRE0);
    /* Send data */
    UDR0 = data;
}

uint8_t UART_ReceiveByte(void) {
    /* Wait for incoming data */
    loop_until_bit_is_set(UCSR0A, RXC0);
    /* Return register value */
    return UDR0;
}

void UART_TransmitString(const char* str) {
    while(*str) {
        UART_TransmitByte((uint8_t)*str);
        str++;
    }
}

/* Converts 4 bits into hexadecimal */
char nibbleToHexCharacter(uint8_t nibble) {
    if (nibble < 10) {
        return ('0' + nibble);
    } else {
        return ('A' + nibble - 10);
    }
}

/* Prints a byte as its hexadecimal equivalent */
void UART_TransmitByteHex(uint8_t byte) {
  uint8_t nibble;
  nibble = (byte & 0xF0) >> 4;
  UART_TransmitByte(nibbleToHexCharacter(nibble));
  nibble = byte & 0x0F;
  UART_TransmitByte(nibbleToHexCharacter(nibble));
}

void DS1302_Init(void) {
    // CE - output, set low
    DS1302_CE_DDR |= (1 << DS1302_CE);
    DS1302_CE_PORT &= ~(1 << DS1302_CE);

    // CLK - output, set low
    DS1302_CLK_DDR |= (1 << DS1302_CLK);
    DS1302_CLK_PORT &= ~(1 << DS1302_CLK);

    // DIO - output, set low (for now)
    DS1302_DIO_DDR |= (1 << DS1302_DIO);
    DS1302_DIO_PORT &= ~(1 << DS1302_DIO);
}

void DS1302_Select(void) {
    // set CE high
    DS1302_CE_PORT |= (1 << DS1302_CE);
}

void DS1302_Deselect(void) {
    // set CE low
    DS1302_CE_PORT &= ~(1 << DS1302_CE);
}

void DS1302_TransmitByte(uint8_t byte) {
    // DIO - output, set low
    DS1302_DIO_DDR |= (1 << DS1302_DIO);
    DS1302_DIO_PORT &= ~(1 << DS1302_DIO);

    // transmit byte, lsb-first
    for(uint8_t i = 0; i < 8; i++) {
        if((byte >> i) & 0x01) {
            // set high
            DS1302_DIO_PORT |= (1 << DS1302_DIO);
        } else {
            // set low
            DS1302_DIO_PORT &= ~(1 << DS1302_DIO);
        }

        // send CLK signal
        DS1302_CLK_PORT |= (1 << DS1302_CLK);
        _delay_us(DS1302_DELAY_USEC);
        DS1302_CLK_PORT &= ~(1 << DS1302_CLK);
    }
}

uint8_t DS1302_ReceiveByte(void) {
    // DIO - input
    DS1302_DIO_DDR &= ~(1 << DS1302_DIO);

    // NB: receive is always done after transmit, thus
    // falling edge of CLK signal was already sent
    // see "Figure 4. Data Transfer Summary" for more details

    // receive byte, lsb-first
    uint8_t byte = 0;
    for(uint8_t i = 0; i < 8; i++) {
        if(DS1302_DIO_PIN & (1 << DS1302_DIO)) {
            byte |= (1 << i);
        }

        // send CLK signal
        DS1302_CLK_PORT |= (1 << DS1302_CLK);
        _delay_us(DS1302_DELAY_USEC);
        DS1302_CLK_PORT &= ~(1 << DS1302_CLK);
    }

    return byte;
}

void set_time() {
    // see "Table 3. Register Address/Definition"
    const uint8_t bytes[8] =
      // sec  min   hour  day    mon  dow(1-7) year  wp (in BCD!)
      { 0x00, 0x11, 0x21, 0x26, 0x04, 0x04,    0x18  0x00 };

    DS1302_Select();
    // 0xBE = clock burst write
    DS1302_TransmitByte(0xBE);
    for(uint8_t i = 0; i < sizeof(bytes); i++) {
        DS1302_TransmitByte(bytes[i]);
    }
    DS1302_Deselect();
}

int main(int argc, char *argv[]) {
    UART_Init();
    DS1302_Init();

    // set current time
    // set_time();

    for (;;) {
        _delay_ms(1000);

        // read current time
        uint8_t bytes[8];

        DS1302_Select();
        // 0xBF = clock burst read
        DS1302_TransmitByte(0xBF);
        for(uint8_t i = 0; i < sizeof(bytes); i++) {
            bytes[i] = DS1302_ReceiveByte();
        }
        DS1302_Deselect();

        // send the time over UART
        UART_TransmitString("Time = ");
        for(uint8_t i = 0; i < sizeof(bytes); i++) {
            UART_TransmitByteHex(bytes[i]);
        }
        UART_TransmitString("\r\n");
    }
}
