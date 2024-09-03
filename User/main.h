#ifndef _MAIN_H
#define _MAIN_H

#include "STC32G.h"
#include "stdio.h"
#include "intrins.h"
#include "string.h"
#include "stdlib.h"

#ifdef __cplusplus
 extern "C" {
#endif


//#define ZGC_DOSE

#define P2P_NH         // ��Ե���� --   ��ʱֻ֧�ֵ�GM��̽ͷ

//��������汾�� 3.0.5��ʼʹ��������ģ��
#define VERSION1 3
#define VERSION2 0
#define VERSION3 5


typedef 	unsigned char	uint8_t;
typedef 	unsigned short	uint16_t;
typedef 	unsigned long	uint32_t;
typedef     unsigned int    uint;    
   
typedef 	unsigned char	BOOL;
#define     TRUE    1
#define     FALSE   0

#define     true    1
#define     false   0

typedef 	unsigned char	BYTE;
typedef 	unsigned short	WORD;
typedef 	unsigned long	DWORD;

typedef 	unsigned char	u8;
typedef 	unsigned short	u16;
typedef 	unsigned long	u32;   

#define bool BYTE

#define alt_u8   BYTE
#define alt_u16 WORD
#define alt_u32 DWORD      
     
#define MAIN_Fosc   11059200UL //6000000UL   //11059200UL      // 11.0592M
//#define MAIN_Fosc        35000000UL    // 35M

#define SysTick     9216   // 10ms    // ��/��, ϵͳ�δ�Ƶ��, ��4000~16000֮��

#define Timer0_Reload   (65536UL - SysTick)     //Timer 0 �ж�Ƶ��

#define FIX_COUNT 11  // ��������
#define NH_COUNT  8   // ��ϵ���

#define HEAD   0x02
#define TAIL   0x03

#define VERSION "V1.0.0"    //�6���ֽ�
   
#define SENSOR_TYPE   0x11                   //TR300̽ͷ
#define POWER_OFF_TIME   (5*60*10)
#define LOW_DIBENDI		0   
#define LOW_GAOBENDI	100
#define HIGH_DIBENDI	0   
#define HIGH_GAOBENDI	50   

#define MIB_CST_DOSERATE_THRESHOLD_ALARM	2	
#define MIB_CST_DOSERATE_THRESHOLD_WARNING	2

#if 0
#define DOSERATE_PRE_ALARM_BIT  (1<<0)		    //������Ԥ��λ
#define DOSERATE_ALARM_BIT      (1<<1)			//�����ʱ���λ
#define DOSE_PRE_ALARM_BIT      (1<<2)			//��������Ԥ��λ
#define DOSE_ALARM_BIT          (1<<3)			//������������λ
#define BATTARY_LOW_BIT         (1<<4)			//��������λ
#define LOW_BACKGROND_BIT       (1<<5)		    //�ͱ��ױ���λ
#define HIGH_BACKGROND_BIT      (1<<6)		    //�߱��ױ���λ
#endif

#define DOSERATE_PRE_ALARM_BIT      (1<<7)      //������Ԥ��λ
#define DOSERATE_ALARM_BIT          (1<<6)      //�����ʱ���λ
#define DOSE_PRE_ALARM_BIT          (1<<5)      //��������Ԥ��λ
#define DOSE_ALARM_BIT              (1<<4)      //������������λ
#define OVER_RANGE_BIT              (1<<3)      //���ر���λ
#define LOW_BACK_BIT                (1<<0)      //�ͱ��ױ���λ

#define Uart485_EN(x) (x)?(P4 |= (1<<6)):(P4 &= ~(1<<6))      // RS485
//#define Light_M(x)     (x)?(P5 |= (1<<2)):(P5 &= ~(1<<2))       // Light

#define LED_R(x)     (x)?(P4 |= (1<<4)):(P4 &= ~(1<<4))       // LED_R
#define LED_G(x)     (x)?(P4 |= (1<<3)):(P4 &= ~(1<<3))       // LED_G
#define LED_Y(x)     (x)?(P4 |= (1<<2)):(P4 &= ~(1<<2))       // LED_Y
#define ALARM(x)     (x)?(P4 |= (1<<1)):(P4 &= ~(1<<1))       // ALARM


#pragma pack(2)
//ϵͳ���ò���
typedef struct
{
    #if 0
  	float  Hv;      // 0x01+��ѹֵ
    float  Z1;      //0x02+�������ֵ1
    float  Ct;      // 0x03+����ʱ��
    float  Hd;      //0x04+��ѹ��
    float  Z2;      // 0x05+�������ֵ2
    float  Z3;      //0x06+�������ֵ3
    float  S1;      // 0x07+̽��������ϵ��1
    float  S2;      //0x08+̽��������ϵ��2
    float  Cr;      // 0x09+У׼����
    uint8_t  Hn;    //0x0A+���ú��غ�
    float  S3;      //0x0B ̽��������ϵ��3
    float  Z4;      //0x0C �������ֵ4

	float PingHuaShiJian;   //ƽ��ʱ��

	float DiYaCanshuA;      //������ͨ��У׼����A	   	  LP
	float DiYaCanshuB;      // ������ͨ��У׼����B       	LP
	float DiYaCanshuC;      // ������ͨ��У׼����C 	        LP
	float GaoYaCanshuA;     //������ͨ��У׼����A 	LP
	float GaoYaCanshuB;     // ������ͨ��У׼����B 	LP
	float GaoYaCanshuC;     // ������ͨ��У׼����C 	LP

//	float BCanshuA;         //��У׼����A   	  LP
//	float BCanshuB;         //��У׼����B  	  LP
//	float BCanshuC;         //��У׼����C        LP
    #endif
   float Canshu1;    // ������ͨ��У׼����C        
   float Canshu2;    // ������ͨ��У׼����C     
   char yuzhi1[4];    // 
   char yuzhi2[4];    // 
   char PingHuaShiJian[4]; // ƽ��ʱ��,���͵�ʱ����Ҫת��Ϊ4�ֽ�ASC�ַ�����
  
}SYS_PRAM;

//������Ϣ
typedef struct
{
    #if 0
  	float  A1;          // 0x01+һ��������ֵ(��)
    float  A2;          //0x02+����������ֵ(��)
    float  Al;          // 0x03+�ͱ��ױ�����ֵ(��)
    float  Ah;          //0x04+�߱��ױ�����ֵ(��)
    float  B1;          // 0x05+һ��������ֵ(��)
    float  B2;          //0x06+����������ֵ(��)
    float  Bl;          // 0x07+�ͱ��ױ�����ֵ(��)
    float  Bh;          //0x08+�߱��ױ�����ֵ(��)
    float  Ac;          // 0x09+alphy���ر���ֵ
    float  Bc;          //0x0A+beta���ر���ֵ
    float  DoseRatePreAlarm;    //0x0B Y������һ������(uSv/h)
    float  DoseRateAlarm;       //0x0C Y�����ʶ�������(uSv/h)
    float  Y3;                  //0x0D Y��������������(uSv/h)
    float  DosePreAlarm;        //0x0E �ۼƼ���Ԥ����ֵ
    float  DoseAlarm;           //0x0F �ۼƼ���������ֵ
    uint8_t   Zu;               //0x10 �ڲ��ۼƼ���������
    #endif

    float  DoseRatePreAlarm;      //������һ������(uSv/h)
    float  DoseRateAlarm;          //�����ʶ�������(uSv/h)
    float  DosePreAlarm;          //�ۼƼ���Ԥ����ֵ
    float  DoseAlarm;              //�ۼƼ���������ֵ
}SYS_ALARM;

//�����ṹ��    LP
typedef struct
{
    #if 0
	float  Dose_B;  uint8_t AjSt;           //0x01+���ڵ���//���ۻ�����
    float  MaxDoseRate_B;  uint8_t HvSt;    //0x02+��ѹ����//����������
    float  C1;  uint8_t C1St;               //0x03+����ֵ1+����״̬��1B������������//C1 ��ͨ�������� LP
    float  C2;  uint8_t C2St;               //0x04+����ֵ2+����״̬��1B�����¼�����//C2 ��ͨ�������� LP
    float  C3;  uint8_t C3St;               // 0x05+����ֵ3+����״̬��1B��(��+��)//�¼�����
    float  P1;  uint8_t P1St;               //0x06+ƽ������ֵ1+����״̬��1B������������//P1��ͨ��ԭʼ���� LP
    float  P2;  uint8_t P2St;               //0x07+ƽ������ֵ2+����״̬��1B������������//P2��ͨ��ԭʼ���� LP
    float  P3;  uint8_t P3St;               //0x08+ƽ������ֵ3+����״̬��1B������������//�¼���
    float  DoseRate;  uint8_t DRSt;         // 0x09+������+����״̬��1B��uSv/h   LP
    float  Dose;  uint8_t DoSt;             // 0x0A+�ۼƼ���+����״̬��1B��uSv   LP
	float  MaxDoseRate;  uint8_t MRSt;	    //��������uSv/h   LP 
    #endif

    float  DoseRate;
    float  Dose;         //�ۻ�����
    
    float WAITINGFORUSE;//����
    DWORD PULSE1;//����1
    DWORD PULSE2;//����2

    /*
    bit7   ������һ��������
    bit6λ �����ʶ���������
    bit5λ �ۼƼ�����һ��������
    bit4λ �ۼƼ����ʶ���������
    bit3λ ���ر�����       
    bit2λ ʧЧһ��������
    bit1λ ʧЧ����������
    bit0λ �ͱ��׹���
    */
    uint8_t State;    //����״̬
    //INF_SOUND_CERTAIN CERTAIN_KEY;//��ť��
    DWORD PULSE3;//����3 -- �ٽ�̽ͷר��

}STU_DOSERATE;

//ϵͳ���ò���
typedef struct
{
    // < 80u
    float DI_A;    //������ͨ��У׼����A        
    float DI_B;    // ������ͨ��У׼����B     
    float DI_C;    //������ͨ��У׼����C    
    #ifdef POE_SEN
    float DI_D;    //������ͨ��У׼����D

    // 80 - 1300
    float MID_A;    //������ͨ��У׼����A        
    float MID_B;    //������ͨ��У׼����B     
    float MID_C;    //������ͨ��У׼����C    
    float MID_D;    //������ͨ��У׼����D  
    #endif
    float GAO_A;    // ������ͨ��У׼����A        
    float GAO_B;    // ������ͨ��У׼����B 
    float GAO_C;    // ������ͨ��У׼����C     
    #ifdef POE_SEN
    float GAO_D;    //������ͨ��У׼����D
    #endif
}SYS_JIAOZHUNYINZI;



#pragma pack()

typedef struct
{
    uint16_t   Sign;  // 0x4142             
    uint8_t    address;//̽ͷ����
    char       VerSion[6];    //�汾��          -- 10
    SYS_PRAM   s_SysParam;       //          -- 20
    SYS_JIAOZHUNYINZI s_Jiaozhun;//У׼����       -- 24
    SYS_ALARM  s_Alarm; 
    float      CoCsRate;            //        -- 4

    float      Fix[FIX_COUNT];                  //  -- 40
    
    uint16_t   ParaCheckSum;//����У���
    
//	uint8_t SensorType;    //̽ͷ����
//	uint8_t res;
//	uint8_t VerSion1;	    //��汾
//	uint8_t VerSion2;	    //С�汾
//	
//	uint8_t VerSion3;	    //SVN��	
//	uint8_t res2;
//	SYS_PRAM s_SysParam;
//
//	SYS_ALARM s_Alarm;
//
//    float Fix[FIX_COUNT];   // С�˴洢������ͨ�ţ�ʹ�õ�ʱ��ת��һ��
//    
//	uint16_t ParaCheckSum;//����У���
}LP_PARAM;

typedef struct
{
    float  CPS1;  //���͵�ʱ��ҪתΪ6�ֽ�asc���뷢��
    float  CPS2;    //���͵�ʱ��ҪתΪ6�ֽ�asc���뷢��
    uint8_t State;    //״̬�֡�����Ϊ��Ч��������Ϊ��Ч������2λ�ֱ��ʾ������һ�������������ʶ����������ۼƼ�����һ���������ۼƼ����ʶ������������ر������ͱ��׹��ϣ�������λδ����
}STU_CPS;


typedef struct
{
    #if 0
  	LP_PARAM	stParam;
    uint8_t SensorType;
	uint8_t isSleep;
	uint8_t isCanReadSensor;
	STU_DOSERATE s_DoseMSG;
	uint16_t DoseRatePreAlarmCnt;           //������Ԥ������������2�α����ܱ���
	uint16_t DoseRateAlarmCnt;              //�����ʱ�������������2�α����ܱ���
	uint32_t NoUartTime;		            //�޴���ͨ��ʱ��
	uint32_t LChannelNoCountTime;			//��ͨ���޼���ʱ��
	uint32_t HChannelNoCountTime;			//��ͨ���޼���ʱ��
	uint8_t	 LowChanneloff;					//��ͨ���Ƿ�رգ��رպ�Ͳ��Ʊ���
	//uint16_t testtime;
    #endif

    LP_PARAM    stParam;
    uint8_t SensorType;
    uint8_t keydownFlag;//�������±�־
    uint8_t keyValue;//������ֵ
    uint32_t keydownTime;//�������¼���ʱ��*10ms
    uint8_t isCanReadSensor;
    STU_DOSERATE s_DoseMSG;
    STU_CPS s_CPS;
    uint8_t range;    //���õ�����
    uint8_t RealRange;//ʵ��ʹ�õ�����
    uint8_t isSwitching;    // >0�����л���λ��
    uint16_t DoseRatePreAlarmCnt;//������Ԥ������������2�α����ܱ���
    uint16_t DoseRateAlarmCnt;//�����ʱ�������������2�α����ܱ���
    uint32_t LChannelNoCountTime;            //��ͨ���޼���ʱ��
    uint32_t HChannelNoCountTime;			//��ͨ���޼���ʱ��
    uint8_t	 LowChanneloff;					//��ͨ���Ƿ�رգ��رպ�Ͳ��Ʊ���
    
}LP_SYSTEM_STTAE;

//��ص����ṹ��
typedef struct
{
	uint16_t Voltage;		    //��ص�ѹֵmv   	LP
	uint8_t  batPercent;		//��ذٷֱ�%		LP
	uint8_t  Status;            //0��������1����ص�ѹ��
}STU_BATTERY;


typedef struct
{
    float cps;
    float dr;
}NH_PARAM;


extern LP_SYSTEM_STTAE SysRunState;

void Error();
void InitParam();

void delay_ms(WORD ms);
void delay_us(BYTE us);
void TimerTask();
void UartHnd();
void Light_Run(u16 dt);


#endif

