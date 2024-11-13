#include "Sensor.h"
#include "mcp4725.h"
#include "system.h"
#include "main.h"
#include "CMD.h"
#include "uart.h"
#include "i2c.h"
#include "flash.h"
#include "CalcDoseRate.h"
#include "DoseRate.h"

LP_SYSTEM_STTAE SysRunState={0};
extern void Adc_Init();
extern uint16_t DeviceGetBatVal(void);
extern void DeviceGetBatAlarm(STU_BATTERY *bat);
extern STU_BATTERY s_Bat;

bool BleReSetFlag = 0;
BYTE BleErrorCnt = 0;
static u8 counter = 0;
static u16 Alarm_cnt = 0;
static u16 Green_cnt = 0;
static u16 Uart_cnt = 0;

//unsigned int ADC16Result = 0; 

//========================================================================
// ��������: void InitParam()
// ��������: ��ʼ����������
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================

void InitParam()
{
    BYTE i;
    
	memset((void*)&SysRunState.stParam,0,sizeof(LP_PARAM));
    SysRunState.stParam.Sign = 0x4142;
    SysRunState.stParam.address = 1;//̽ͷ��ַ
    SysRunState.stParam.s_SysParam.Canshu1 = 1;
    SysRunState.stParam.s_SysParam.Canshu2 = 1;

	SysRunState.stParam.s_Jiaozhun.DI_A = 1.06581410364015E-14;                 //������ͨ��У׼����
	SysRunState.stParam.s_Jiaozhun.DI_B = 0.291478787484166;                    //������ͨ��У׼����
	SysRunState.stParam.s_Jiaozhun.DI_C = 1;                                     //������ͨ��У׼����
	SysRunState.stParam.s_Jiaozhun.GAO_A = 54590.764135567;                     //������ͨ��У׼����
	SysRunState.stParam.s_Jiaozhun.GAO_B = 5.67266663489089;                    //������ͨ��У׼����
	SysRunState.stParam.s_Jiaozhun.GAO_C = 1;                                   //������ͨ��У׼����
	
	SysRunState.stParam.s_Alarm.DosePreAlarm = 300;            //300uSv
	SysRunState.stParam.s_Alarm.DoseAlarm = 400;               //400uSv
	SysRunState.stParam.s_Alarm.DoseRatePreAlarm = 300;        //300uSv/h
	SysRunState.stParam.s_Alarm.DoseRateAlarm = 400;           //400uSv/h

    sprintf((char*)SysRunState.stParam.s_SysParam.yuzhi1,"150");
    SysRunState.stParam.CoCsRate = 1.19;

    for (i=0;i<FIX_COUNT;i++)
    {
        SysRunState.stParam.Fix[i] = FloatToSmall(  1.0);
    }
    
	WritePara();
}

void DevInit(void)
{
    GDoseSeg = LOW_SEG;
}

void Error()
{
    while(1)
    {
        //Light_M(1);
        delay_ms(50);
        //Light_M(0);
        delay_ms(50);
    }
}

//========================================================================
// ��������: void delay_ms(WORD ms) _11MHz 
// ��������: ������ʱ����
// ��ڲ���: @WORD ms����ʱ���ٺ���
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void delay_ms(WORD ms)              
{
    WORD t = 1000;
    while(ms--)
    {
        for (t=0;t<1000;t++) ;
    }
}

#if 0
void delay_ms(WORD ms)	//@6.000MHz
{
	DWORD i;
    while(ms--)
    {
    	_nop_();
    	_nop_();
    	_nop_();
        i = 1498UL;
    	while (i) i--;
    }
}
#endif

void delay_us(BYTE us)
{
    while(us--)
    {
        ;
    }
}

//========================================================================
// ��������: void IoInit()
// ��������: ��Ƭ��I/O�ڳ�ʼ��
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void IoInit()
{
    EAXFR = 1;
    WTST = 0;                       //���ó���ָ����ʱ��������ֵΪ0�ɽ�CPUִ��ָ����ٶ�����Ϊ���

    P0M1 = 0x50;   P0M0 = 0x00;     //����Ϊ׼˫���           P0M1 = 0x50
    P1M1 = 0x00;   P1M0 = 0x00;     //����Ϊ׼˫���           P1M1 = 0x02    
    P2M1 = 0x00;   P2M0 = 0x00;     //����Ϊ׼˫���
    P3M1 = 0x00;   P3M0 = 0x00;     //P3.3����Ϊ�������
    P4M1 = 0x00;   P4M0 = 0x5E;    
    P5M1 = 0x00;   P5M0 = 0x00;    
}

void AlmTask()
{
     if((SysRunState.s_DoseMSG.State&OVER_RANGE_BIT))//̽�����߹���
    {
        LED_R(1);
        if(SysRunState.keyValue == 0)
        {
            ALARM(1);
        }
        else
        {
            ALARM(0);
        }
        LED_Y(0);
        LED_G(0);
    }
    else if((SysRunState.s_DoseMSG.State&LOW_BACK_BIT))//̽�����쳣
    {
        LED_Y(1);
        if(SysRunState.keyValue == 0)
        {
            ALARM(1);
        }
        else
        {
            ALARM(0);
        }
        LED_R(0);
        LED_G(0);
    }
    else if((SysRunState.s_DoseMSG.State&DOSERATE_ALARM_BIT))//��������
    {
        if((Alarm_cnt < 20))//||((Alarm_cnt > 20)&&(Alarm_cnt < 35)))
        {
            LED_R(1);
        }
        else
        {
            LED_R(0);
            LED_Y(0);
        }
        if((Alarm_cnt > 80)&&(Alarm_cnt < 90)&&(SysRunState.keyValue == 0))
        {
              ALARM(1);
        }
        else
        {
              ALARM(0);
        }
        if(Alarm_cnt > 150)
        Alarm_cnt = 0;
        LED_G(0);
    }
    else if(SysRunState.s_DoseMSG.State & DOSERATE_PRE_ALARM_BIT)//һ������
    {
        if(Alarm_cnt < 20)
        {
            LED_Y(1);
        }
        else
        {
            LED_R(0);
            LED_Y(0);
        }
        if((Alarm_cnt > 100)&&(Alarm_cnt < 130)&&(SysRunState.keyValue == 0))
        {
              ALARM(1);
        }
        else
        {
              ALARM(0);
        }
        LED_Y(0);
    }
    else
    {
        LED_R(0);
        LED_Y(0);
        ALARM(0);
        if(Green_cnt < 20)
        {
            LED_G(1);
        }
        else
        {
            LED_G(0);
        }
    }
}


//========================================================================
// ��������: void TimerTask()
// ��������: ��ʱ����ͨ����ʱ��0��ʱ10ms�������������
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void TimerTask()
{
    u16 delta = 0;
    static u16 Time1s = 0;
    if(Timer0Cnt)
    {
        delta = Timer0Cnt * 10;
        Timer0Cnt = 0;
        if(RX1_Cnt>0)
        {
            Rx1_Timer += delta;
        }
        if(RX3_Cnt>0)
        {
            Rx3_Timer += delta;
        }
        Alarm_cnt += delta;
        Green_cnt += delta;
        if(Green_cnt >= 5000)
        {
            Green_cnt = 0;
        }
        if(Alarm_cnt >= 2000)
        {
            Alarm_cnt = 0;
        }
        if(Uart_cnt <= 3000)
        {
            Uart_cnt += delta;
            if(Uart_cnt > 3000)
            {
                uart485_send("UartSend",8);
                Uart_cnt = 3100;
            }
        }
        
        Time1s += delta;
        if(Time1s >= 1000)                      //100*10=1000ms
        {         
            CLR_WDT = 1;  // ι��
            Time1s = 0;
            SysRunState.isCanReadSensor = 1;
        }
        //AlmTask();
    }
}

//========================================================================
// ��������: void BleHnd()
// ��������: ͨ��BLE����λ������
// ��ڲ���: @��
// ��������: ��
// ��ǰ�汾: VER1.0
// �޸�����: 2023.5.5
// ��ǰ����:
// ������ע: 
//========================================================================
void UartHnd()
{
//    if(revFlag)
//    {
        if(Rx1_Timer > 20)                  //���ڳ�ʱ20ms
        {
            Rx1_Timer = 0;
           
            //DataPro(RX1_Buffer,RX1_Cnt);
            //printf("UART1\r\n");
            uart485_send(RX1_Buffer,RX1_Cnt);
           
            ClearUartBuf();  
//            revFlag = 0;
        }
//    }
}

#if 0
void Light_Run(u16 dt)
{
    static u16 counter = 0;
    //u16 compare = 5000;
    counter += dt;
    if(counter > 5000)
    {
        counter = 0;
        Light_M(0);
    }
    else if(counter >= (5000 - 100))
    {      
        Light_M(1);
    }
}
#endif

int main(void)
{    
    SysInit();
    IoInit();
    //checkApp();
	DevInit();
    delay_ms(200);
    
    Timer0_Init();
    delay_ms(200);
	Timer3_Init();
    delay_ms(200);
	Timer4_Init();
    delay_ms(200);
    
    Uart1_Init();
    ClearUartBuf();
    delay_ms(500);
    
    Uart3_Init();
    ClearRs485Buf();
    delay_ms(500);
    
	GetPara(&SysRunState.stParam);
    delay_ms(1000);
    
    SensorInit();
  
	EA = 1;

	SensorMeasureBegin();//��ʼ���� 
	InitArr();
    MCP4725_OutVol(MCP4725_S1_ADDR,(char *)SysRunState.stParam.s_SysParam.yuzhi1);
	delay_ms(100);
    WDT_CONTR |= (1<<5) |  7;  // �������Ź���Լ8��
    while(1)
    {   
        TimerTask();                   

        if(SysRunState.isCanReadSensor == 1)
        {
            CaptureSensorPluseCounter(); //����ǰ�������
            SensorMeasureBegin();         //��ʼ���� 
            SysRunState.isCanReadSensor = 0;
        }

        //UartHnd();
        Uart3Hnd();
        
    }
}




