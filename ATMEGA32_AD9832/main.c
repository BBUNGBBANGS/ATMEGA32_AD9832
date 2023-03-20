
#define F_CPU 7372800UL

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>

#define OUTPUT_ON(x, y) (x |= (1 << y))  // x의 y 비트를 설정(1)
#define OUTPUT_OFF(x, y) (x &= ~(1 << y)) // x의 y 비트를 클리어(0)
#define INPUT_READ(x, y) (x & ( 1<< y)) // x의 y 비트를 read

#define AD9832_PORT         (PORTA)
#define AD9832_FSYNC_PIN    (5)
#define AD9832_SDI_PIN      (6)
#define AD9832_CLK          (7)
#define AD9832_MASTER_CLOCK (10000000UL)

#define AD5204_PORT         (PORTA)
#define AD5204_CS_PIN       (4)
#define AD5204_SDI_PIN      (6)
#define AD5204_CLK          (7)

#define AD7705_PORT         (PORTC)
#define AD7705_PIN          (PINC)
#define AD7705_OUT_PIN      (0)
#define AD7705_CS_PIN       (1)
#define AD7705_CLK_PIN      (2)
#define AD7705_IN_PIN       (3)

static const uint8_t REG_CMM = 0x0; //communication register 8 bit
static const uint8_t REG_SETUP = 0x1; //setup register 8 bit
static const uint8_t REG_CLOCK = 0x2; //clock register 8 bit
static const uint8_t REG_DATA = 0x3; //data register 16 bit, contains conversion result
static const uint8_t REG_TEST = 0x4; //test register 8 bit, POR 0x0
static const uint8_t REG_NOP = 0x5; //no operation
static const uint8_t REG_OFFSET = 0x6; //offset register 24 bit
static const uint8_t REG_GAIN = 0x7; // gain register 24 bit

static const uint8_t CHN_AIN1 = 0x0; //AIN1; calibration register pair 0
static const uint8_t CHN_AIN2 = 0x1; //AIN2; calibration register pair 1

//output update rate
//CLK FS1 FS0
static const uint8_t UPDATE_RATE_20 = 0x0; // 20 Hz
static const uint8_t UPDATE_RATE_25 = 0x1; // 25 Hz
static const uint8_t UPDATE_RATE_100 = 0x2; // 100 Hz
static const uint8_t UPDATE_RATE_200 = 0x3; // 200 Hz
static const uint8_t UPDATE_RATE_50 = 0x4; // 50 Hz
static const uint8_t UPDATE_RATE_60 = 0x5; // 60 Hz
static const uint8_t UPDATE_RATE_250 = 0x6; // 250 Hz
static const uint8_t UPDATE_RATE_500 = 0x7; // 500 Hz

//operating mode options
//MD1 MD0
static const uint8_t MODE_NORMAL = 0x0; //normal mode
static const uint8_t MODE_SELF_CAL = 0x1; //self-calibration
static const uint8_t MODE_ZERO_SCALE_CAL = 0x2; //zero-scale system calibration, POR 0x1F4000, set FSYNC high before calibration, FSYNC low after calibration
static const uint8_t MODE_FULL_SCALE_CAL = 0x3; //full-scale system calibration, POR 0x5761AB, set FSYNC high before calibration, FSYNC low after calibration

//gain setting
static const uint8_t GAIN_1 = 0x0;
static const uint8_t GAIN_2 = 0x1;
static const uint8_t GAIN_4 = 0x2;
static const uint8_t GAIN_8 = 0x3;
static const uint8_t GAIN_16 = 0x4;
static const uint8_t GAIN_32 = 0x5;
static const uint8_t GAIN_64 = 0x6;
static const uint8_t GAIN_128 = 0x7;

static const uint8_t UNIPOLAR = 0x0;
static const uint8_t BIPOLAR = 0x1;

static const uint8_t CLK_DIV_1 = 0x1;
static const uint8_t CLK_DIV_2 = 0x2;

uint16_t Reference_Voltage = 3300; //3.3[V]
uint16_t AIN1_Voltage,AIN2_Voltage;

static void Port_Init(void);
static void Uart_Init(void);
static void AD9832_Init(void);
static void AD9832_Write_Register(uint16_t reg);
static void AD9832_Set_Frequency(uint32_t freq);
static void AD9832_Sleep(uint8_t enable);
static void AD5204_Init(void);
static void AD5204_Write_Register(uint8_t channel, uint8_t val);
static void AD5204_Set_Scale(uint8_t level);
static void AD7705_Init(void);
static void AD7705_Reset(void);
static void AD7705_Channel_Setting(uint8_t channel, uint8_t clkDivider, uint8_t polarity, uint8_t gain, uint8_t updRate);
static void AD7705_Write_Communication_Register(uint8_t reg, uint8_t channel, uint8_t readWrite);
static void AD7705_Write_Clock_Register(uint8_t CLKDIS, uint8_t CLKDIV, uint8_t UpdateRate);
static void AD7705_Write_Setup_Register(uint8_t operationMode, uint8_t gain, uint8_t unipolar, uint8_t buffered, uint8_t fsync);
static uint8_t AD7705_Data_Ready(uint8_t channel);
static uint16_t AD7705_ADC_Result(uint8_t channel, uint16_t offset);
static uint32_t AD7705_Read_Data(uint8_t length);
static void AD7705_Write_Data(uint8_t val,uint8_t length);
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
    AD7705_Init();
    while (1) 
    {
        /* AD5204 저항 증폭 Scale 설정 */
        AD5204_Set_Scale(255);        
        /* AD9832 Analog 출력 주파수[Hz] 설정 */
        AD9832_Set_Frequency(500);        
        /* AIN1 채널 ADC값 Read , Resolution : 1[mV] ex) 1000 -> 1[v] */
        AIN1_Voltage = AD7705_ADC_Result(CHN_AIN1,0);
        /* AIN2 채널 ADC값 Read , Resolution : 1[mV] ex) 1000 -> 1[v] */
        AIN2_Voltage = AD7705_ADC_Result(CHN_AIN2,0);

        /* Uart 출력 */
		sprintf(tx_buf,"Channel1 ADC : %d.%d[V] , Channel2 ADC : %d.%d[V] \n",(AIN1_Voltage/1000),(AIN1_Voltage%100),(AIN2_Voltage/1000),(AIN2_Voltage%100));
		Uart_Transmit_Array(tx_buf,strlen(tx_buf));
		Clear_Tx_Buffer(tx_buf,100);
        _delay_ms(50);
    }
}

static void Port_Init(void)
{
    /* PORTA 출력 Default LOW 설정*/
    PORTA = 0x00; 
    /* PA4~7 핀 출력 설정*/
    DDRA = 0xF0;     
    /* PORTC 출력 Default LOW 설정*/
    PORTC = 0x00;
    /* PC1~3 핀 출력 설정, PC0 핀 입력 설정 */
    DDRA = 0xF0;     
    DDRC = 0x0E;
}

static void AD7705_Init(void)
{
    /* CS 핀 / SCLK 핀 Default HIGH 설정 */
    OUTPUT_ON(AD7705_PORT,AD7705_CS_PIN);        
    OUTPUT_ON(AD7705_PORT,AD7705_CLK_PIN);    
    AD7705_Reset();
    /* AIN1 채널 ADC 셋팅*/
    AD7705_Channel_Setting(CHN_AIN1,CLK_DIV_1, BIPOLAR, GAIN_1, UPDATE_RATE_25);
    /* AIN2 채널 ADC 셋팅*/
    AD7705_Channel_Setting(CHN_AIN2,CLK_DIV_1, BIPOLAR, GAIN_1, UPDATE_RATE_25);
}

static void AD7705_Reset(void)
{
    /* 32bit High 전송할 경우 리셋 */
    AD7705_Write_Data(0xFF,4);    
    AD7705_Write_Data(0xFF,4);
}

static void AD7705_Channel_Setting(uint8_t channel, uint8_t clkDivider, uint8_t polarity, uint8_t gain, uint8_t updRate)
{
    /* 채널 별 Clock 레지스터 설정 */
    AD7705_Write_Communication_Register(REG_CLOCK, channel, 0);
    /* 채널 별 Clock divider 및 Update Rate 설정 */
    AD7705_Write_Clock_Register(0, clkDivider, updRate);

    /* 채널 별 Setup 레지스터 설정 */
    AD7705_Write_Communication_Register(REG_SETUP, channel, 0);
    /* 채널 별 Calibration 설정 및 Gain , Polarity 설정 */
    AD7705_Write_Setup_Register(MODE_SELF_CAL, gain, polarity, 0, 0);
    
    /* 설정 완료까지 대기 */
    while (!AD7705_Data_Ready(channel));
}

//write communication register
//   7        6      5      4      3      2      1      0
//0/DRDY(0) RS2(0) RS1(0) RS0(0) R/W(0) STBY(0) CH1(0) CH0(0)
static void AD7705_Write_Communication_Register(uint8_t reg, uint8_t channel, uint8_t readWrite)
{
    uint8_t val = 0;
    /* 레지스터 버퍼 설정 */
    val = (reg << 4) | (readWrite << 3) | channel;
    /* 8-bit 레지스터 Write */
    AD7705_Write_Data(val,1);
}

//Clock Register
//   7      6       5        4        3        2      1      0
//ZERO(0) ZERO(0) ZERO(0) CLKDIS(0) CLKDIV(0) CLK(1) FS1(0) FS0(1)
//
//CLKDIS: master clock disable bit
//CLKDIV: clock divider bit
static void AD7705_Write_Clock_Register(uint8_t CLKDIS, uint8_t CLKDIV, uint8_t UpdateRate)
{
    uint8_t val = 0;
    /* 레지스터 버퍼 설정 */
    val = (CLKDIS << 4) | (CLKDIV << 3) | UpdateRate;
    /* Clock 값 있을 경우 초기화(설정 금지) */
    val &= ~(1 << 2); 
    /* 8-bit 레지스터 Write */
    AD7705_Write_Data(val,1);
}

//Setup Register
//  7     6     5     4     3      2      1      0
//MD10) MD0(0) G2(0) G1(0) G0(0) B/U(0) BUF(0) FSYNC(1)
static void AD7705_Write_Setup_Register(uint8_t operationMode, uint8_t gain, uint8_t unipolar, uint8_t buffered, uint8_t fsync) 
{
    uint8_t val = 0;
    /* 레지스터 버퍼 설정 */
    val = (operationMode << 6) | (gain << 3) | (unipolar << 2) | (buffered << 1) | fsync;
    /* 8-bit 레지스터 Write */
    AD7705_Write_Data(val,1);
}

static uint8_t AD7705_Data_Ready(uint8_t channel) 
{
    uint8_t val = 0;
    /* Command 레지스터 출력 설정 */
    AD7705_Write_Communication_Register(REG_CMM, channel, 1);
    /* 8bit LOW 출력 및 Read */
    val = (uint8_t)AD7705_Read_Data(1);
    /* DRDY 핀 Status 리턴 */
    return (val & 0x80) == 0x0;
}

static uint16_t AD7705_ADC_Result(uint8_t channel, uint16_t offset)
{
    uint16_t retval = 0;
    /* 전송 가능할 때 까지 대기 */
    while (!AD7705_Data_Ready(channel));
    /* Data 레지스터 read 설정 */
    AD7705_Write_Communication_Register(REG_DATA, channel, 1);
    /* 16bit Data Read */
    retval = AD7705_Read_Data(2);
    /* ADC 전압 계산 Resolution 1[mV] */
    retval = (uint16_t)((float)retval * (float)Reference_Voltage / 65536.0f - offset);

    return retval;
}

static uint32_t AD7705_Read_Data(uint8_t length)
{
    uint32_t retval = 0;
    /* CS Pin LOW 설정 */
    OUTPUT_OFF(AD7705_PORT,AD7705_CS_PIN);    
    _delay_us(20);
    for(uint8_t i = 0; i < (8 * length); i++)
    {
        /* SCLK 핀 LOW 설정 */
        OUTPUT_OFF(AD7705_PORT,AD7705_CLK_PIN);    
        _delay_us(20);
        /* SCLK 핀 HIGH 설정 */
        OUTPUT_ON(AD7705_PORT,AD7705_CLK_PIN); 
        /* SCLK Rising Edge에서 Data Read */
        if(INPUT_READ(AD7705_PIN,AD7705_OUT_PIN) != 0)
        {
            retval |= 1 << ((8*length - 1) - i);
        }
        _delay_us(20);
    }
    /* CS Pin HIGH 설정 */
    OUTPUT_ON(AD7705_PORT,AD7705_CS_PIN);    

    return retval;
}

static void AD7705_Write_Data(uint8_t val,uint8_t length)
{
    /* CS Pin LOW 설정 */
    OUTPUT_OFF(AD7705_PORT,AD7705_CS_PIN);    
    _delay_us(20);
    for(uint8_t i = 0; i < (8 * length); i++)
    {        
        /* SCLK 핀 LOW 설정 */
        OUTPUT_OFF(AD7705_PORT,AD7705_CLK_PIN);    
        _delay_us(10);
        /* MSB 출력 (최상위 bit가 먼저 출력 됨)*/
        if (((val >> ((8*length - 1) - i)) & 0x01) == 1)
        {
            OUTPUT_ON(AD7705_PORT,AD7705_IN_PIN);            
        }
        else
        {
            OUTPUT_OFF(AD7705_PORT,AD7705_IN_PIN);      
        }
        _delay_us(10);        
        /* SCLK 핀 HIGH 설정 */
        OUTPUT_ON(AD7705_PORT,AD7705_CLK_PIN); 
        _delay_us(20);
    }
    /* CS Pin HIGH 설정 */
    OUTPUT_ON(AD7705_PORT,AD7705_CS_PIN);    
}

static void AD9832_Init(void)
{
    /* FSYNC 및 SCLK 핀 HIGH 설정 */
    OUTPUT_ON(AD9832_PORT,AD9832_FSYNC_PIN);    
    OUTPUT_ON(AD9832_PORT,AD9832_CLK);
    _delay_ms(10);
    /* 초기 레지스터값 Write */
    AD9832_Write_Register(0xF800);
    AD9832_Write_Register(0xB000);
    AD9832_Write_Register(0xC000);
}

static void AD9832_Set_Frequency(uint32_t freq)
{
    /* Master Clock 증폭 값 계산 */
    uint32_t multiplier = (uint32_t)(((uint64_t)freq << 32) / AD9832_MASTER_CLOCK);

    /* Master Clock 증폭 값 4-Byte로 분할 */
    uint16_t data_0,data_1,data_2,data_3;
    data_3 = 0x3300 | (0x00FF & (uint16_t)(multiplier >> 24));  // FREQ0 H-MSBs
    data_2 = 0x2200 | (0x00FF & (uint16_t)(multiplier >> 16));  // FREQ0 L-MSBs
    data_1 = 0x3100 | (0x00FF & (uint16_t)(multiplier >> 8));   // FREQ0 H-LSBs
    data_0 = 0x2000 | (0x00FF & (uint16_t)(multiplier));        // FREQ0 L-LSBs    

    /* MSB 4-Byte 전송 */
    AD9832_Write_Register(data_3);
    AD9832_Write_Register(data_2);
    AD9832_Write_Register(data_1);
    AD9832_Write_Register(data_0);

    /* Phase 0 값 전송 */
    AD9832_Write_Register(0x1900);  // PHASE0 MSBs
    AD9832_Write_Register(0x0800);  // PHASE0 LSBs    
}

static void AD9832_Sleep(uint8_t enable)
{
    if(enable == 1)
    {
        /* Sleep 모드 진입 */
        AD9832_Write_Register(0xE000);
    }
    else
    {
        /* Nornal 모드 진입 */
        AD9832_Write_Register(0xC000);        
    }
}

static void AD9832_Write_Register(uint16_t reg)
{
    /* SCLK Pin HIGH 설정 */
    OUTPUT_ON(AD9832_PORT,AD9832_CLK);
    _delay_us(20);    
    /* FSYNC Pin LOW 설정 */
    OUTPUT_OFF(AD9832_PORT,AD9832_FSYNC_PIN);

    for (uint8_t i = 0; i < 16; i++)
    {
        /* MSB 출력 (최상위 bit가 먼저 출력 됨)*/
        /* SCLK Rising Edge에서 출력값 변경 */
        if (((reg >> (15-i)) & 0x01) == 1)
        {
            OUTPUT_ON(AD9832_PORT,AD9832_SDI_PIN);
        }
        else
        {
            OUTPUT_OFF(AD9832_PORT,AD9832_SDI_PIN);
        }
        _delay_us(50);    
        /* SCLK Pin LOW 설정 */
        OUTPUT_OFF(AD9832_PORT,AD9832_CLK);
        _delay_us(50);
        /* SCLK Pin HIGH 설정 */
        OUTPUT_ON(AD9832_PORT,AD9832_CLK);
    }
    /* FSYNC Pin HIGH 설정 */
    OUTPUT_ON(AD9832_PORT,AD9832_FSYNC_PIN);
}

static void AD5204_Init(void)
{
    /* CS Pin HIGH 설정 */
    OUTPUT_ON(AD5204_PORT,AD5204_CS_PIN);
    /* default scale 설정 */
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
    /* SCLK Pin LOW 설정 */
    OUTPUT_OFF(AD5204_PORT,AD5204_CLK);
    _delay_us(20);
    /* CS Pin LOW 설정 */
    OUTPUT_OFF(AD5204_PORT,AD5204_CS_PIN);

    for (uint8_t i = 0; i < 11; i++)
    {
        /* SCLK Pin LOW 설정 */
        OUTPUT_ON(AD5204_PORT,AD5204_CLK);
        /* 8~11 bit 채널 설정값 출력 */
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
        /* 0~7 bit data값 출력 */
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
        /* SCLK Pin LOW 설정 */
        OUTPUT_OFF(AD9832_PORT,AD5204_CLK);
        _delay_us(50);
    }
    /* CS Pin LOW 설정 */
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