#include "main.h"
#include "usart.h"
#include <stdio.h>
#include "tjc_usart_hmi.h"

typedef struct
{
    uint16_t Head;
    uint16_t Tail;
    uint16_t Length;
    uint8_t  Ring_data[RINGBUFFER_LEN];
}RingBuffer_t;

RingBuffer_t ringBuffer;	
uint8_t RxBuffer[1];

void intToStr(int num, char* str) {
    int i = 0;
    int isNegative = 0;

    if (num < 0) {
        isNegative = 1;
        num = -num;
    }

    do {
        str[i++] = (num % 10) + '0';
        num /= 10;
    } while (num);

    if (isNegative) {
        str[i++] = '-';
    }

    str[i] = '\0';

    int start = 0;
    int end = i - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
    return ;
}

void uart_send_char(char ch)
{
	uint8_t ch2 = (uint8_t)ch;

	HAL_UART_Transmit(&TJC_UART, &ch2, 1, 0xffff);
	return;
}

void uart_send_string(char* str)
{
    
    while(*str!=0&&str!=0)
    {
        
        uart_send_char(*str++);
    }
	return;
}

void tjc_send_string(char* str)
{
    
    while(*str!=0&&str!=0)
    {
        
        uart_send_char(*str++);
    }
	uart_send_char(0xff);
	uart_send_char(0xff);
	uart_send_char(0xff);
	return;
}

void tjc_send_txt(char* objname, char* attribute, char* txt)
{

    uart_send_string(objname);
    uart_send_char('.');
    uart_send_string(attribute);
    uart_send_string("=\"");
    uart_send_string(txt);
    uart_send_char('\"');
	uart_send_char(0xff);
	uart_send_char(0xff);
	uart_send_char(0xff);
	return;
}

void tjc_send_wave(const char *tjc_wave_name, int *data_buf, int len, uint8_t reverse)
{
    char cmd_buf[64];

    sprintf(cmd_buf, "addt %s.id,0,%d",tjc_wave_name , len);

    tjc_send_string(cmd_buf);
    HAL_Delay(10);

    for (int i = 0; i < len; i++)
    {
        uint8_t byte_data;
        if (reverse == 0)
        {
            byte_data = data_buf[i];         
        }
        else
        {
            byte_data = data_buf[len - 1 - i] ; 
        }
        uart_send_char(byte_data);
    }

    uart_send_char(0x01);
    uart_send_char(0xFF);
    uart_send_char(0xFF);
    uart_send_char(0xFF);
}

void tjc_send_val(char* objname, char* attribute, int val)
{
	
    uart_send_string(objname);
    uart_send_char('.');
    uart_send_string(attribute);
    uart_send_char('=');
    
    char txt[12]="";
    intToStr(val, txt);
    uart_send_string(txt);
	uart_send_char(0xff);
	uart_send_char(0xff);
	uart_send_char(0xff);
	return;
}

void tjc_send_nstring(char* str, unsigned char str_length)
{
    
    for (int var = 0; var < str_length; ++var)
    {
        
        uart_send_char(*str++);
    }
	uart_send_char(0xff);
	uart_send_char(0xff);
	uart_send_char(0xff);
	return;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == TJC_UART_INS)	
	{
		write1ByteToRingBuffer(RxBuffer[0]);
		HAL_UART_Receive_IT(&TJC_UART,RxBuffer,1);		
	}
	
	return;
}

void initRingBuffer(void)
{
	
	ringBuffer.Head = 0;
	ringBuffer.Tail = 0;
	ringBuffer.Length = 0;
	return;
}

void write1ByteToRingBuffer(uint8_t data)
{
	if(ringBuffer.Length >= RINGBUFFER_LEN) 
	{
	return ;
	}
	ringBuffer.Ring_data[ringBuffer.Tail]=data;
	ringBuffer.Tail = (ringBuffer.Tail+1)%RINGBUFFER_LEN;
	ringBuffer.Length++;
	return ;
}

void deleteRingBuffer(uint16_t size)
{
	if(size >= ringBuffer.Length)
	{
	    initRingBuffer();
	    return;
	}
	for(int i = 0; i < size; i++)
	{
		ringBuffer.Head = (ringBuffer.Head+1)%RINGBUFFER_LEN;
		ringBuffer.Length--;
		
	}
return;
}

uint8_t read1ByteFromRingBuffer(uint16_t position)
{
	uint16_t realPosition = (ringBuffer.Head + position) % RINGBUFFER_LEN;

	return ringBuffer.Ring_data[realPosition];
}

uint16_t getRingBufferLength()
{
	return ringBuffer.Length;
}

uint8_t isRingBufferOverflow()
{
	return ringBuffer.Length < RINGBUFFER_LEN;
}
