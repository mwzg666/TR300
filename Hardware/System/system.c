#include "main.h"
#include "uart.h"
#include "system.h"
#include "CMD.h"
u16  Timer0Cnt = 0;
u32  Pluse3Cnt = 0;
u32  Pluse4Cnt = 0;
u16  IDLCnt = 0;

u8   LowOverFlowCnt3 = 0;
u8   HightOverFlowCnt4 = 0;

void SysInit()
{
    HIRCCR = 0x80;           // �����ڲ�����IRC
    while(!(HIRCCR & 1));
    CLKSEL = 0x00; 
}


//========================================================================
// ��������: void Timer0_Init(void)	
// ��������: ��ʱ��ԴΪ11.0592MHz��TIMER0��ʼ��Ϊ10ms��ʱ
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void Timer0_Init(void)		//100����@6MHz
{
//	AUXR &= 0x7F;			//��ʱ��ʱ��12Tģʽ----6MHz
//	TMOD &= 0xF0;			//���ö�ʱ��ģʽ
//	TL0 = 0x78;				//���ö�ʱ��ʼֵ����6MHz_10ms
//	TH0 = 0xEC;				//���ö�ʱ��ʼֵ

//    TL0 = 0xB0;				//���ö�ʱ��ʼֵ����6MHz_100ms
//	TH0 = 0x3C;				//���ö�ʱ��ʼֵ
    
//	TF0 = 0;				//���TF0��־
//	TR0 = 1;				//��ʱ��0��ʼ��ʱ
//	ET0 = 1;				//ʹ�ܶ�ʱ��0�ж�


    AUXR |= 0x00;    //Timer0 set as 12T, 16 bits timer auto-reload, _11MHz_10ms
    TH0 = (u8)(Timer0_Reload / 256);
    TL0 = (u8)(Timer0_Reload % 256);
    ET0 = 1;    //Timer0 interrupt enable
    TR0 = 1;    //Tiner0 run   
    // �ж����ȼ�3
    PT0  = 0;
    PT0H = 0;

}

//========================================================================
// ��������: void Timer3_Init(void)
// ��������: TIMER3��ʼ��Ϊ�½��ؼ���ģʽ
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void Timer3_Init(void)
{
    T3L = 0x00;
    T3H = 0x00;
    T4T3M = 0x0C;
    IE2 = 0x20;             //Timer3 interrupt enable
}



//========================================================================
// ��������: void Timer4_Init(void)
// ��������: TIMER4��ʼ��Ϊ�½��ؼ���ģʽ
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.18
// ��ǰ����:
// ������ע: 
//========================================================================
void Timer4_Init(void)
{
    T4L = 0x00;
    T4H = 0x00;
    T4T3M |= 0xC0;
    IE2 |= 0x40;             //Timer4 interrupt enable

}

//========================================================================
// ��������: void Timer1_Init(void)	
// ��������: ��ʱ��ԴΪ11.0592MHz��TIMER1��ʼ��Ϊ10ms��ʱ
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void Timer1_Init(void)		//10����@11.0592MHz
{
	AUXR &= 0xBF;			//��ʱ��ʱ��12Tģʽ
	TMOD &= 0x0F;			//���ö�ʱ��ģʽ
	TL1 = 0x00;				//���ö�ʱ��ʼֵ
	TH1 = 0xDC;				//���ö�ʱ��ʼֵ
	TF1 = 0;				//���TF1��־
	TR1 = 1;				//��ʱ��1��ʼ��ʱ
	ET1 = 1;				//ʹ�ܶ�ʱ��1�ж�
}
 
//========================================================================
// ��������: u32 GetCounter(void)
// ��������: ��ȡTIMER3���������
// ��ڲ���: @��
// ��������: ���������ֵ
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
u32 GetCounter(void)
{
  	u32 Return = 0;
    Pluse3Cnt =((u32)LowOverFlowCnt3*65536) | (u32)((u32)T3H*256+T3L) ;
    Return =Pluse3Cnt;
    T3R = 0;
    T3H = 0;
    T3L = 0;
    T3R = 1;
    LowOverFlowCnt3 = 0;
	Pluse3Cnt = 0;
	return Return;
}
u32 GetHightCounter(void)
{
  	u32 Return = 0;
    Pluse4Cnt = ((u32)HightOverFlowCnt4*65536) | (u32)((u32)T4H*256+T4L) ;
    Return = Pluse4Cnt;

    T4R = 0;
    T4H = 0;
    T4L = 0;
    T4R = 1;
    HightOverFlowCnt4 = 0;
	Pluse4Cnt = 0;
	return Return;
}

void Timer0_Isr (void) interrupt 1   //Timer0�ж�
{
    Timer0Cnt ++;
    IDLCnt++;
    
}

void Timer3_Isr() interrupt 19      //Timer3�ж�
{
    LowOverFlowCnt3++;
   
}

void Timer4_Isr() interrupt 20      //Timer4�ж�
{
    HightOverFlowCnt4++;
   
}



