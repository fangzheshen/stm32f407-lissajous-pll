#ifndef __TJCUSARTHMI_H__
#define __TJCUSARTHMI_H__

#include <stdio.h>
#include "main.h"

#define TJC_UART huart1
#define TJC_UART_INS USART1

void tjc_send_string(char* str);
void tjc_send_txt(char* objname, char* attribute, char* txt);
void tjc_send_val(char* objname, char* attribute, int val);
void tjc_send_nstring(char* str, unsigned char str_length);
void initRingBuffer(void);
void write1ByteToRingBuffer(uint8_t data);
void deleteRingBuffer(uint16_t size);
uint16_t getRingBufferLength(void);
uint8_t read1ByteFromRingBuffer(uint16_t position);
void tjc_send_wave(const char *tjc_wave_name, int *data_buf, int len, uint8_t reverse);
void uart_send_char(char ch);

#define RINGBUFFER_LEN	(500)     

#define usize getRingBufferLength()
#define code_c() initRingBuffer()
#define udelete(x) deleteRingBuffer(x)
#define u(x) read1ByteFromRingBuffer(x)
#define FRAME_LENGTH 7
extern uint8_t RxBuffer[1];
extern uint32_t msTicks;

#endif
