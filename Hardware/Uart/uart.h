#ifndef UART_H
#define UART_H
#include "main.h"

//===============================��������=============================
#define Uart_BaudRate       9600

#define MAX_LENGTH          200

#define Baudrate   115200UL
#define Baudrate3   9600UL

#define SYSTEM_FREQ         25000000

#define UART_MAX_BAUDRATE   115200    //���崮��������С������
#define UART_MIN_BAUDRATE   1200




//===============================�ṹ�嶨��===========================



//===============================��������=============================
extern u8  TX1_Cnt;                 //���ͼ���
extern u8  RX1_Cnt;                 //���ռ���
extern bit B_TX1_Busy;              //����æ��־
extern u16 Rx1_Timer;
extern u8  RX1_Buffer[MAX_LENGTH];  //���ջ���
extern u8  TX3_Cnt;                 //���ͼ���
extern u8  RX3_Cnt;                 //���ռ���
extern bit B_TX3_Busy;              //����æ��־
extern u16 Rx3_Timer;
extern u8  RX3_Buffer[MAX_LENGTH];  //���ջ���
extern u8 revFlag;

//===============================��������=============================
void Uart1_Init(void);
void Uart3_Init(void);
void uart485_send(u8 *buf, u8 len);
void Uart_send(u8 *buf, u8 len);
void Uart3Hnd(void);
void ClearRs485Buf();
void ClearUartBuf();

#endif