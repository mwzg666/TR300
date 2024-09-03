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

#define P2P_NH         // 点对点拟合 --   暂时只支持单GM管探头

//定义软件版本号 3.0.5开始使用新蓝牙模块
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

#define SysTick     9216   // 10ms    // 次/秒, 系统滴答频率, 在4000~16000之间

#define Timer0_Reload   (65536UL - SysTick)     //Timer 0 中断频率

#define FIX_COUNT 11  // 修正段数
#define NH_COUNT  8   // 拟合点数

#define HEAD   0x02
#define TAIL   0x03

#define VERSION "V1.0.0"    //最长6个字节
   
#define SENSOR_TYPE   0x11                   //TR300探头
#define POWER_OFF_TIME   (5*60*10)
#define LOW_DIBENDI		0   
#define LOW_GAOBENDI	100
#define HIGH_DIBENDI	0   
#define HIGH_GAOBENDI	50   

#define MIB_CST_DOSERATE_THRESHOLD_ALARM	2	
#define MIB_CST_DOSERATE_THRESHOLD_WARNING	2

#if 0
#define DOSERATE_PRE_ALARM_BIT  (1<<0)		    //剂量率预警位
#define DOSERATE_ALARM_BIT      (1<<1)			//剂量率报警位
#define DOSE_PRE_ALARM_BIT      (1<<2)			//剂量当量预警位
#define DOSE_ALARM_BIT          (1<<3)			//剂量当量报警位
#define BATTARY_LOW_BIT         (1<<4)			//电量报警位
#define LOW_BACKGROND_BIT       (1<<5)		    //低本底报警位
#define HIGH_BACKGROND_BIT      (1<<6)		    //高本底报警位
#endif

#define DOSERATE_PRE_ALARM_BIT      (1<<7)      //剂量率预警位
#define DOSERATE_ALARM_BIT          (1<<6)      //剂量率报警位
#define DOSE_PRE_ALARM_BIT          (1<<5)      //剂量当量预警位
#define DOSE_ALARM_BIT              (1<<4)      //剂量当量报警位
#define OVER_RANGE_BIT              (1<<3)      //过载报警位
#define LOW_BACK_BIT                (1<<0)      //低本底报警位

#define Uart485_EN(x) (x)?(P4 |= (1<<6)):(P4 &= ~(1<<6))      // RS485
//#define Light_M(x)     (x)?(P5 |= (1<<2)):(P5 &= ~(1<<2))       // Light

#define LED_R(x)     (x)?(P4 |= (1<<4)):(P4 &= ~(1<<4))       // LED_R
#define LED_G(x)     (x)?(P4 |= (1<<3)):(P4 &= ~(1<<3))       // LED_G
#define LED_Y(x)     (x)?(P4 |= (1<<2)):(P4 &= ~(1<<2))       // LED_Y
#define ALARM(x)     (x)?(P4 |= (1<<1)):(P4 &= ~(1<<1))       // ALARM


#pragma pack(2)
//系统配置参数
typedef struct
{
    #if 0
  	float  Hv;      // 0x01+高压值
    float  Z1;      //0x02+甄别器阈值1
    float  Ct;      // 0x03+计数时间
    float  Hd;      //0x04+高压误
    float  Z2;      // 0x05+甄别器阈值2
    float  Z3;      //0x06+甄别器阈值3
    float  S1;      // 0x07+探测器修正系数1
    float  S2;      //0x08+探测器修正系数2
    float  Cr;      // 0x09+校准因子
    uint8_t  Hn;    //0x0A+启用核素号
    float  S3;      //0x0B 探测器修正系数3
    float  Z4;      //0x0C 甄别器阈值4

	float PingHuaShiJian;   //平滑时间

	float DiYaCanshuA;      //低量程通道校准因子A	   	  LP
	float DiYaCanshuB;      // 低量程通道校准因子B       	LP
	float DiYaCanshuC;      // 低量程通道校准因子C 	        LP
	float GaoYaCanshuA;     //高量程通道校准因子A 	LP
	float GaoYaCanshuB;     // 高量程通道校准因子B 	LP
	float GaoYaCanshuC;     // 高量程通道校准因子C 	LP

//	float BCanshuA;         //β校准因子A   	  LP
//	float BCanshuB;         //β校准因子B  	  LP
//	float BCanshuC;         //β校准因子C        LP
    #endif
   float Canshu1;    // 低量程通道校准因子C        
   float Canshu2;    // 高量程通道校准因子C     
   char yuzhi1[4];    // 
   char yuzhi2[4];    // 
   char PingHuaShiJian[4]; // 平滑时间,发送的时候需要转换为4字节ASC字符发送
  
}SYS_PRAM;

//报警信息
typedef struct
{
    #if 0
  	float  A1;          // 0x01+一级报警阈值(α)
    float  A2;          //0x02+二级报警阈值(α)
    float  Al;          // 0x03+低本底报警阈值(α)
    float  Ah;          //0x04+高本底报警阈值(α)
    float  B1;          // 0x05+一级报警阈值(β)
    float  B2;          //0x06+二级报警阈值(β)
    float  Bl;          // 0x07+低本底报警阈值(β)
    float  Bh;          //0x08+高本底报警阈值(β)
    float  Ac;          // 0x09+alphy核素报警值
    float  Bc;          //0x0A+beta核素报警值
    float  DoseRatePreAlarm;    //0x0B Y剂量率一级报警(uSv/h)
    float  DoseRateAlarm;       //0x0C Y剂量率二级报警(uSv/h)
    float  Y3;                  //0x0D Y剂量率三级报警(uSv/h)
    float  DosePreAlarm;        //0x0E 累计剂量预警阀值
    float  DoseAlarm;           //0x0F 累计剂量报警阀值
    uint8_t   Zu;               //0x10 内部累计剂量柱报警
    #endif

    float  DoseRatePreAlarm;      //剂量率一级报警(uSv/h)
    float  DoseRateAlarm;          //剂量率二级报警(uSv/h)
    float  DosePreAlarm;          //累计剂量预警阀值
    float  DoseAlarm;              //累计剂量报警阀值
}SYS_ALARM;

//计数结构体    LP
typedef struct
{
    #if 0
	float  Dose_B;  uint8_t AjSt;           //0x01+正在调整//β累积剂量
    float  MaxDoseRate_B;  uint8_t HvSt;    //0x02+高压故障//β最大剂量率
    float  C1;  uint8_t C1St;               //0x03+计数值1+报警状态（1B）（α计数）//C1 低通道剂量率 LP
    float  C2;  uint8_t C2St;               //0x04+计数值2+报警状态（1B）（β计数）//C2 高通道剂量率 LP
    float  C3;  uint8_t C3St;               // 0x05+计数值3+报警状态（1B）(α+β)//β剂量率
    float  P1;  uint8_t P1St;               //0x06+平滑计数值1+报警状态（1B）（α计数）//P1低通道原始计数 LP
    float  P2;  uint8_t P2St;               //0x07+平滑计数值2+报警状态（1B）（α计数）//P2高通道原始计数 LP
    float  P3;  uint8_t P3St;               //0x08+平滑计数值3+报警状态（1B）（α计数）//β计数
    float  DoseRate;  uint8_t DRSt;         // 0x09+剂量率+报警状态（1B）uSv/h   LP
    float  Dose;  uint8_t DoSt;             // 0x0A+累计剂量+报警状态（1B）uSv   LP
	float  MaxDoseRate;  uint8_t MRSt;	    //最大剂量率uSv/h   LP 
    #endif

    float  DoseRate;
    float  Dose;         //累积剂量
    
    float WAITINGFORUSE;//备用
    DWORD PULSE1;//脉冲1
    DWORD PULSE2;//脉冲2

    /*
    bit7   剂量率一级报警、
    bit6位 剂量率二级报警、
    bit5位 累计剂量率一级报警、
    bit4位 累计剂量率二级报警、
    bit3位 过载报警、       
    bit2位 失效一级报警、
    bit1位 失效二级报警、
    bit0位 低本底故障
    */
    uint8_t State;    //报警状态
    //INF_SOUND_CERTAIN CERTAIN_KEY;//按钮报
    DWORD PULSE3;//脉冲3 -- 临界探头专用

}STU_DOSERATE;

//系统配置参数
typedef struct
{
    // < 80u
    float DI_A;    //低量程通道校准因子A        
    float DI_B;    // 低量程通道校准因子B     
    float DI_C;    //低量程通道校准因子C    
    #ifdef POE_SEN
    float DI_D;    //低量程通道校准因子D

    // 80 - 1300
    float MID_A;    //中量程通道校准因子A        
    float MID_B;    //中量程通道校准因子B     
    float MID_C;    //中量程通道校准因子C    
    float MID_D;    //中量程通道校准因子D  
    #endif
    float GAO_A;    // 高量程通道校准因子A        
    float GAO_B;    // 高量程通道校准因子B 
    float GAO_C;    // 高量程通道校准因子C     
    #ifdef POE_SEN
    float GAO_D;    //高量程通道校准因子D
    #endif
}SYS_JIAOZHUNYINZI;



#pragma pack()

typedef struct
{
    uint16_t   Sign;  // 0x4142             
    uint8_t    address;//探头类型
    char       VerSion[6];    //版本号          -- 10
    SYS_PRAM   s_SysParam;       //          -- 20
    SYS_JIAOZHUNYINZI s_Jiaozhun;//校准因子       -- 24
    SYS_ALARM  s_Alarm; 
    float      CoCsRate;            //        -- 4

    float      Fix[FIX_COUNT];                  //  -- 40
    
    uint16_t   ParaCheckSum;//参数校验和
    
//	uint8_t SensorType;    //探头类型
//	uint8_t res;
//	uint8_t VerSion1;	    //大版本
//	uint8_t VerSion2;	    //小版本
//	
//	uint8_t VerSion3;	    //SVN号	
//	uint8_t res2;
//	SYS_PRAM s_SysParam;
//
//	SYS_ALARM s_Alarm;
//
//    float Fix[FIX_COUNT];   // 小端存储，方便通信，使用的时候转换一下
//    
//	uint16_t ParaCheckSum;//参数校验和
}LP_PARAM;

typedef struct
{
    float  CPS1;  //发送的时候要转为6字节asc编码发送
    float  CPS2;    //发送的时候要转为6字节asc编码发送
    uint8_t State;    //状态字‘１’为有效，‘０’为无效，７—2位分别表示剂量率一级报警、剂量率二级报警、累计剂量率一级报警、累计剂量率二级报警、过载报警、低本底故障，１、０位未定义
}STU_CPS;


typedef struct
{
    #if 0
  	LP_PARAM	stParam;
    uint8_t SensorType;
	uint8_t isSleep;
	uint8_t isCanReadSensor;
	STU_DOSERATE s_DoseMSG;
	uint16_t DoseRatePreAlarmCnt;           //剂量率预警次数，连续2次报警能报警
	uint16_t DoseRateAlarmCnt;              //剂量率报警次数，连续2次报警能报警
	uint32_t NoUartTime;		            //无串口通信时间
	uint32_t LChannelNoCountTime;			//低通道无计数时间
	uint32_t HChannelNoCountTime;			//高通道无计数时间
	uint8_t	 LowChanneloff;					//低通道是否关闭，关闭后就不计报警
	//uint16_t testtime;
    #endif

    LP_PARAM    stParam;
    uint8_t SensorType;
    uint8_t keydownFlag;//按键按下标志
    uint8_t keyValue;//按键键值
    uint32_t keydownTime;//按键按下计数时间*10ms
    uint8_t isCanReadSensor;
    STU_DOSERATE s_DoseMSG;
    STU_CPS s_CPS;
    uint8_t range;    //设置的量程
    uint8_t RealRange;//实际使用的量程
    uint8_t isSwitching;    // >0正在切换档位，
    uint16_t DoseRatePreAlarmCnt;//剂量率预警次数，连续2次报警能报警
    uint16_t DoseRateAlarmCnt;//剂量率报警次数，连续2次报警能报警
    uint32_t LChannelNoCountTime;            //低通道无计数时间
    uint32_t HChannelNoCountTime;			//高通道无计数时间
    uint8_t	 LowChanneloff;					//低通道是否关闭，关闭后就不计报警
    
}LP_SYSTEM_STTAE;

//电池电量结构体
typedef struct
{
	uint16_t Voltage;		    //电池电压值mv   	LP
	uint8_t  batPercent;		//电池百分比%		LP
	uint8_t  Status;            //0：正常，1：电池电压低
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

