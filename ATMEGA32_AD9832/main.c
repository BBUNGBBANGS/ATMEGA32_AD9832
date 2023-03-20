
#define F_CPU 7372800UL

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>

#define OUTPUT_ON(x, y) (x |= (1 << y))  // x의 y 비트를 설정(1)
#define OUTPUT_OFF(x, y) (x &= ~(1 << y)) // x의 y 비트를 클리어(0)

#define AD9832_PORT         (PORTA)
#define AD9832_FSYNC_PIN    (5)
#define AD9832_SDI_PIN      (6)
#define AD9832_CLK          (7)
#define AD9832_MASTER_CLOCK (10000000UL)

#define AD5204_PORT         (PORTA)
#define AD5204_CS_PIN       (4)
#define AD5204_SDI_PIN      (6)
#define AD5204_CLK          (7)

static void Port_Init(void);
static void Uart_Init(void);
static void AD9832_Init(void);
static void AD9832_Write_Register(uint16_t reg);
static void AD9832_Set_Frequency(uint32_t freq);
static void AD9832_Sleep(uint8_t enable);
static void AD5204_Init(void);
static void AD5204_Write_Register(uint8_t channel, uint8_t val);
static void AD5204_Set_Scale(uint8_t level);
static void Uart_Transmit(char data);
static void Uart_Transmit_Array(char *data_ptr, unsigned char length);
static void Clear_Tx_Buffer(char *buf,unsigned int length);
int main(void)
{
    char tx_buf[100] = {0,};
    Port_Init();
    Uart_Init();

    AD9832_Init();
    AD5204_Init();
    while (1) 
    {
        AD5204_Set_Scale(255);
        AD9832_Set_Frequency(500);
		sprintf(tx_buf,"<Uart Print Test> \n");
		Uart_Transmit_Array(tx_buf,strlen(tx_buf));
		Clear_Tx_Buffer(tx_buf,100);
        _delay_ms(10);
    }
}

static void Port_Init(void)
{
    PORTA = 0x00; // PA Output LOW
    DDRA = 0xF0; // PA4~7 Output Direction
    PORTC = 0x00;
    DDRC = 0x07l
}

static void AD9832_Init(void)
{
    OUTPUT_ON(AD9832_PORT,AD9832_FSYNC_PIN);    
    OUTPUT_ON(AD9832_PORT,AD9832_CLK);
    _delay_ms(10);
    AD9832_Write_Register(0xF800);
    AD9832_Write_Register(0xB000);
    AD9832_Write_Register(0xC000);
}

static void AD9832_Set_Frequency(uint32_t freq)
{
    // Calculate the multiplier
    uint32_t multiplier = (uint32_t)(((uint64_t)freq << 32) / AD9832_MASTER_CLOCK);

    // Break the multiplier into four chunks that specify each of the four registers that must be written to
    uint16_t data_0,data_1,data_2,data_3;
    data_3 = 0x3300 | (0x00FF & (uint16_t)(multiplier >> 24));  // FREQ0 H-MSBs
    data_2 = 0x2200 | (0x00FF & (uint16_t)(multiplier >> 16));  // FREQ0 L-MSBs
    data_1 = 0x3100 | (0x00FF & (uint16_t)(multiplier >> 8));   // FREQ0 H-LSBs
    data_0 = 0x2000 | (0x00FF & (uint16_t)(multiplier));        // FREQ0 L-LSBs    

    AD9832_Write_Register(data_3);
    AD9832_Write_Register(data_2);
    AD9832_Write_Register(data_1);
    AD9832_Write_Register(data_0);

    AD9832_Write_Register(0x1900);  // PHASE0 MSBs
    AD9832_Write_Register(0x0800);  // PHASE0 LSBs    
}

static void AD9832_Sleep(uint8_t enable)
{
    if(enable == 1)
    {
        AD9832_Write_Register(0xE000);
    }
    else
    {
        AD9832_Write_Register(0xC000);        
    }
}

static void AD9832_Write_Register(uint16_t reg)
{
    OUTPUT_ON(AD9832_PORT,AD9832_CLK);
    _delay_us(20);
    OUTPUT_OFF(AD9832_PORT,AD9832_FSYNC_PIN);

    for (uint8_t i = 0; i < 16; i++)
    {
        if (((reg >> (15-i)) & 0x01) == 1)
        {
            OUTPUT_ON(AD9832_PORT,AD9832_SDI_PIN);
        }
        else
        {
            OUTPUT_OFF(AD9832_PORT,AD9832_SDI_PIN);
        }
        _delay_us(50);
        OUTPUT_OFF(AD9832_PORT,AD9832_CLK);
        _delay_us(50);
        OUTPUT_ON(AD9832_PORT,AD9832_CLK);
    }

    OUTPUT_ON(AD9832_PORT,AD9832_FSYNC_PIN);
}

static void AD5204_Init(void)
{
    OUTPUT_ON(AD5204_PORT,AD5204_CS_PIN);
    AD5204_Set_Scale(4);
}

static void AD5204_Set_Scale(uint8_t level)
{
    for(uint8_t channel = 0; channel < 4; channel++)
    {
        AD5204_Write_Register(channel,level);
    }    
}

static void AD5204_Write_Register(uint8_t channel, uint8_t val)
{
    _delay_us(20);
    OUTPUT_OFF(AD5204_PORT,AD5204_CLK);
    _delay_us(20);
    OUTPUT_OFF(AD5204_PORT,AD5204_CS_PIN);

    for (uint8_t i = 0; i < 11; i++)
    {
        OUTPUT_ON(AD5204_PORT,AD5204_CLK);
        if(i<3)
        {
            if (((channel >> (2-i)) & 0x01) == 1)
            {
                OUTPUT_ON(AD5204_PORT,AD5204_SDI_PIN);
            }
            else
            {
                OUTPUT_OFF(AD5204_PORT,AD5204_SDI_PIN);
            }
        }
        else
        {
            if (((val >> (10-i)) & 0x01) == 1)
            {
                OUTPUT_ON(AD5204_PORT,AD5204_SDI_PIN);
            }
            else
            {
                OUTPUT_OFF(AD5204_PORT,AD5204_SDI_PIN);
            }
        }

        _delay_us(50);
        OUTPUT_OFF(AD9832_PORT,AD5204_CLK);
        _delay_us(50);
    }
    OUTPUT_ON(AD5204_PORT,AD5204_CS_PIN);
}

static void Uart_Init(void)
{
	UCSRB= (1<<RXEN) | (1<<TXEN);                  // Enable Receiver and Transmitter
	UCSRC= (1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0);   // Asynchronous mode 8-bit data and 1-stop bit
	UCSRA= 0x00;                                   // Clear the UASRT status register
	UBRRL = 3; //115200bps 3(7.3728Mhz)
	UBRRH = 0; //115200bps 3(7.3728Mhz)
}

static void Uart_Transmit_Array(char *data_ptr, unsigned char length)
{
    unsigned char ch;
    for(ch=0;ch<length;ch++)
    {
        Uart_Transmit(*data_ptr++);
    }
}

static void Uart_Transmit(char data)
{
    /* Wait for empty transmit buffer */
    while (!( UCSRA & (1<<UDRE)));
    /* Put data into buffer, sends the data */
    UDR = data;
}
static void Clear_Tx_Buffer(char *buf,unsigned int length)
{
    for(unsigned int i = 0; i < length; i++)
    {
        buf[i] = 0;
    }
}