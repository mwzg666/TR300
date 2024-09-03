#include <stdlib.h>
#include "CMD.h"
#include "sensor.h"
#include "mcp4725.h"
#include "flash.h"
#include "uart.h"
#include "mwpro.h"
#include "CalcCPS.h"
#include "Mcp4725.h"
#include "crc.h"

#define FRAM_DELAY delay_ms(100)

STU_CMD s_Head={0x68};
uint8_t SendBuf[100];

extern void DeviceGetBatAlarm(STU_BATTERY *bat);


//========================================================================
// 函数名称: WORD WordToSmall(WORD dat)
// 函数功能: 将WORD的数据转换为小端模式
// 入口参数: @WORD dat：要转换的数据
// 函数返回: 返回类型为WORD的小端模式数据
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================

WORD WordToSmall(WORD dat)
{
	BYTE buf[2];
    BYTE t;
    WORD ret;
    
    memcpy(buf, &dat, 2);
	t = buf[1];
	buf[1] = buf[0];
	buf[0] = t;
	
    memcpy(&ret, buf, 2);
    return ret;
}

float FloatToSmall(float dat)
{
	BYTE buf[4];
    BYTE t;
    float ret;
    
    memcpy(buf, &dat, 4);
	t = buf[3];
	buf[3] = buf[0];
	buf[0] = t;
	t = buf[2];
	buf[2] = buf[1];
	buf[1] = t;

    memcpy(&ret, buf, 4);
    return ret;
}

DWORD DwordToSmall(DWORD dat)
{
	BYTE buf[4];
    BYTE t;
    DWORD ret;
    
    memcpy(buf, &dat, 4);
	t = buf[3];
	buf[3] = buf[0];
	buf[0] = t;
	t = buf[2];
	buf[2] = buf[1];
	buf[1] = t;

    memcpy(&ret, buf, 4);
    return ret;
}


//========================================================================
// 函数名称: void GetPara(LP_PARAM *me)
// 函数功能: 从FLASH中读取参数，包括"控制参数"和"报警参数"
// 入口参数: @me：数据
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================
void GetPara(LP_PARAM *me)
{ 
  	EEPROM_read(0,(u8 *)me,sizeof(LP_PARAM));
    if ( (SysRunState.stParam.ParaCheckSum !=  CheckSum((BYTE *)&SysRunState.stParam,sizeof(LP_PARAM)-2))
        ||(SysRunState.stParam.Sign != 0x4142) )
	{
	  	InitParam();
	}
    memcpy(SysRunState.stParam.VerSion,VERSION,6);

}

//========================================================================
// 函数名称: void WritePara()
// 函数功能: 写入数据到内存中
// 入口参数: @无
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================
void WritePara()
{
	EA = 0;
    EEPROM_SectorErase(0);
    EEPROM_SectorErase(512);
	SysRunState.stParam.ParaCheckSum = CheckSum((BYTE *)&SysRunState.stParam,sizeof(LP_PARAM)-2);//add by kevin at 20150417
	if (!EEPROM_write(0, (u8 *)&SysRunState.stParam, sizeof(LP_PARAM)))
    {
        Error();
    }	
	EA = 1;
}


//向上位机发送命令
void SendData(uint8_t cmd, uint8_t *cdata, uint8_t length)
{
    uint8_t sumCheck=0,i;
    s_Head.head = 0x02;
    s_Head.length = length + 8;
    s_Head.type = SENSOR_TYPE;//SysRunState.SensorType+0x11;//0x11<->310; 0x12<->TR320; 0x13<->TR330
    s_Head.channel = SysRunState.stParam.address;
	s_Head.cmd = cmd;
	memcpy(SendBuf,(uint8_t*)&s_Head,sizeof(STU_CMD));
	if(length>0)
	{
		memcpy(&SendBuf[sizeof(STU_CMD)],cdata,length);
	}

    for(i=0;i<length+4;i++)
   {
         sumCheck += SendBuf[i+1];
   }
   sprintf((char*)&SendBuf[length+sizeof(STU_CMD)],"%02X",sumCheck);
   SendBuf[length+sizeof(STU_CMD)+2] = 3;
   
   Uart_send(SendBuf,(u8)(length+8));

    
//	crc = CRC16(snedbuf,length+sizeof(STU_CMD));
//    crc = WordToSmall(crc);
//	memcpy(&snedbuf[length+sizeof(STU_CMD)],(uint8_t*)&crc,2);
//	snedbuf[length+sizeof(STU_CMD)+2] = 0x16;
//	
//	uartble_send(snedbuf,(u8)(length+7));
}

/*******************************************************************************
功能：获取上位机指定的模式
输入：无
输出：工作模式
*******************************************************************************/
int GetWorkMode(void)
{
  return 3;
}

void ReadFix()
{
    SendData(CMD_READ_FIX,(uint8_t*)&SysRunState.stParam.Fix,sizeof(float)*FIX_COUNT);
}

void WriteFix(BYTE *dat)
{
    memcpy((uint8_t*)&SysRunState.stParam.Fix,dat,sizeof(float)*FIX_COUNT);
    
    SendData(CMD_WRITE_FIX,NULL,0);
    SaveParam();
}

void AlarmConfirm()
{
    SysRunState.s_DoseMSG.Dose = 0;   // 清累计剂量
    SysRunState.s_DoseMSG.State = 0; // 清报警状态
    
    SendData('E',NULL,0);
}



/*******************************************************************************
功能：命令分析与执行
输入：U32 id:指令ID号
      unsigned char dlc: 数据长度
      unsigned char *cdata:数据指针
输出：无
*******************************************************************************/
STU_CMD gs_CMD={0};
void DataPro(uint8_t *cdata, uint8_t length)
{  
  	uint16_t i,j;
    uint8_t checkOut = 0;
    char checkOutbuf[3] = {0};
	for(i=0;i<length; i++)
	{
	  	if(cdata[i] == HEAD)
		{
		  	memcpy(&gs_CMD,&cdata[i],sizeof(STU_CMD));
            //gs_CMD.length = WordToSmall(gs_CMD.length);
			 if((gs_CMD.length > length)||(cdata[gs_CMD.length-1] != TAIL)
               ||(!((gs_CMD.channel == SysRunState.stParam.address)||(gs_CMD.channel == 0xff)))  
                 )
            {
                //continue;
                return;    
            }

             for(j=0; j< gs_CMD.length-4; j++)
            {
                  checkOut += cdata[i+j+1];
            }    
            sprintf(checkOutbuf,"%02X",checkOut);
            //printf("checkOutbuf = %s\r\n",checkOutbuf);

            if(memcmp(checkOutbuf,&cdata[i+gs_CMD.length]-3,2) == 0)
            {
                //LED_G(1);
    			switch(gs_CMD.cmd)
    			{
    				case 'C':  ACK_CMD_C();break;               // 联络命
    				case 'V':  ACK_CMD_V();break;               // 读计数
    				
                    case 'P':  ACK_CMD_P();    break;           // 读脉冲
                    case 'H':  ACK_CMD_H(cdata[i+5]);    break; //设置探测器地址
                    case 'B':  ACK_CMD_B(&cdata[i+5]);break;    // 写报警参数
                    case 'W':  ACK_CMD_W(&cdata[i+5]); break;   // 写参数
                    case 'F':  ACK_CMD_F();break;               // 读报警参数
                    case 'R':  ACK_CMD_R();break;               // 读参数
                    case 'T':  ACK_CMD_T();break;               // 测温
                    case 'Q':  ACK_CMD_Q(cdata[i+5]);break;     // 量程查询或者设置
                    case 'M':  ACK_CMD_M();break;               // 软件版本
                    case 'J':  ACK_CMD_J();break;               // 读校准因子
                    case 'N':  ACK_CMD_N(&cdata[i+5]);break;    // 修改校准因子
                    case 'X':  ReadCS(); break;                 // 读Co/Cs比
                    case 'Y':  SetCS(&cdata[i+5]); break;       // 写Co/Cs比
                    case 'E':  AlarmConfirm();    break;        // 报警确认

                    case CMD_READ_FIX:  ReadFix(); break;
                    case CMD_WRITE_FIX: WriteFix(&cdata[i+5]);  break;
    				
    				case 0x28://asm(" mov &0xFFBE, PC;"); //跳转到升级代码//远程升级
    				  break; 	
    				 
    			    default: break;
    			 
    			}
	  	    }
			i += (gs_CMD.length+4);
		}
	}
}

void ReadCS()
{
    float Cocs;
    Cocs = FloatToSmall(SysRunState.stParam.CoCsRate);
    SendData('X',(uint8_t*)&Cocs,4);
}

void SetCS(u8 *cs)
{
    float Cocs;
       
    memcpy(&Cocs,cs, 4);  
    SysRunState.stParam.CoCsRate = FloatToSmall(Cocs); 
    SendData('Y',NULL,0);
    SaveParam();
}  


/*******************************************************************************
功能：联络命令响应
输入：unsigned char SensorType:传感器类型
输出：无
*******************************************************************************/
void ACK_CMD_C(void)
{
	SendData('C',NULL,0);
}

/*******************************************************************************
功能：设置探测器号
输入：
输出：
*******************************************************************************/

void ACK_CMD_H(uint8_t Address)
{
    SysRunState.stParam.address = Address;
    SendData('H',NULL,0);
    SaveParam();
}


/*******************************************************************************
功能：读命令(R)响应
输入：无
输出：无
*******************************************************************************/
void ACK_CMD_R(void)
{
    SYS_PRAM ps;
    
    ps.Canshu1 = FloatToSmall(SysRunState.stParam.s_SysParam.Canshu1);
    ps.Canshu2 = FloatToSmall(SysRunState.stParam.s_SysParam.Canshu2);
    memcpy(ps.yuzhi1,SysRunState.stParam.s_SysParam.yuzhi1,4);
    memcpy(ps.yuzhi2,SysRunState.stParam.s_SysParam.yuzhi2,4);
    memcpy(ps.PingHuaShiJian,SysRunState.stParam.s_SysParam.PingHuaShiJian,4);  
    SendData('R',(uint8_t*)&ps,sizeof(SYS_PRAM));
}


/*******************************************************************************
功能：写参数命令(W)响应
输入：unsigned char *cdata:参数
输出：无
*******************************************************************************/
void ACK_CMD_W(unsigned char *cdata)
{
    SYS_PRAM wcm;
    memcpy((uint8_t*)&wcm,cdata,sizeof(SYS_PRAM));
       
    SysRunState.stParam.s_SysParam.Canshu1 = FloatToSmall(wcm.Canshu1);
    SysRunState.stParam.s_SysParam.Canshu2 = FloatToSmall(wcm.Canshu2);

    memcpy(SysRunState.stParam.s_SysParam.yuzhi1,wcm.yuzhi1,4);
    memcpy(SysRunState.stParam.s_SysParam.yuzhi2,wcm.yuzhi2,4);
    memcpy(SysRunState.stParam.s_SysParam.PingHuaShiJian,wcm.PingHuaShiJian,4);

    MCP4725_OutVol(MCP4725_S1_ADDR,SysRunState.stParam.s_SysParam.yuzhi1);

	SendData('W',NULL,0);
    SaveParam();

}

/*******************************************************************************
功能： 读校准因子(J)响应
输入：unsigned char SensorType:传感器类型
输出：无
*******************************************************************************/
void ACK_CMD_J(void)
{
    SYS_JIAOZHUNYINZI Jzyz_r;
    Jzyz_r.DI_A = FloatToSmall(SysRunState.stParam.s_Jiaozhun.DI_A);
    Jzyz_r.DI_B = FloatToSmall(SysRunState.stParam.s_Jiaozhun.DI_B);
    Jzyz_r.DI_C = FloatToSmall(SysRunState.stParam.s_Jiaozhun.DI_C);
    Jzyz_r.GAO_A = FloatToSmall(SysRunState.stParam.s_Jiaozhun.GAO_A);
    Jzyz_r.GAO_B = FloatToSmall(SysRunState.stParam.s_Jiaozhun.GAO_B);
    Jzyz_r.GAO_C = FloatToSmall(SysRunState.stParam.s_Jiaozhun.GAO_C);
    
    SendData('J',(uint8_t*)&Jzyz_r,sizeof(SYS_JIAOZHUNYINZI));
}

/*******************************************************************************
功能：写校准因子(M)响应
输入：unsigned char SensorType:传感器类型
输出：无
*******************************************************************************/
void ACK_CMD_N(unsigned char *cdata)
{
    SYS_JIAOZHUNYINZI Jzyz_w;   
    memcpy((uint8_t*)&Jzyz_w,cdata,sizeof(SYS_JIAOZHUNYINZI));
    SysRunState.stParam.s_Jiaozhun.DI_A = FloatToSmall(Jzyz_w.DI_A);
    SysRunState.stParam.s_Jiaozhun.DI_B = FloatToSmall(Jzyz_w.DI_B);
    SysRunState.stParam.s_Jiaozhun.DI_C = FloatToSmall(Jzyz_w.DI_C);
    SysRunState.stParam.s_Jiaozhun.GAO_A = FloatToSmall(Jzyz_w.GAO_A);
    SysRunState.stParam.s_Jiaozhun.GAO_B = FloatToSmall(Jzyz_w.GAO_B);
    SysRunState.stParam.s_Jiaozhun.GAO_C = FloatToSmall(Jzyz_w.GAO_C);

    //SysRunState.stParam.s_SysParam.Canshu1 = SysRunState.stParam.s_Jiaozhun.DI_C;
    //SysRunState.stParam.s_SysParam.Canshu2 = SysRunState.stParam.s_Jiaozhun.GAO_C;
    SendData('N',NULL,0);
    SaveParam();
    //RefreshParam();
}

/*******************************************************************************
功能：读计数命令(V)响应
输入：unsigned char SensorType:传感器类型
输出：无
*******************************************************************************/

void ACK_CMD_V(void)
{ 
    STU_DOSERATE gs_Dose;

    gs_Dose.Dose = FloatToSmall(SysRunState.s_DoseMSG.Dose);
    gs_Dose.DoseRate = FloatToSmall(SysRunState.s_DoseMSG.DoseRate);
    gs_Dose.State = SysRunState.s_DoseMSG.State;
    //gs_Dose.MaxDoseRate = FloatToSmall(SysRunState.s_DoseMSG.MaxDoseRate);
    
    gs_Dose.PULSE1 = DwordToSmall(SysRunState.s_CPS.CPS1);
    gs_Dose.PULSE2 = DwordToSmall(SysRunState.s_CPS.CPS2);
    gs_Dose.PULSE3 = 0;
    
	SendData('V',(uint8_t*)&gs_Dose,sizeof(STU_DOSERATE));
}

/*******************************************************************************
功能：读计数命令(V)响应
输入：unsigned char SensorType:传感器类型
输出：无
*******************************************************************************/
void ACK_CMD_P(void)
{
    char buf[13];
    
    sprintf(buf,"%06ld",(long)SysRunState.s_CPS.CPS1);
    sprintf(&buf[6],"%06ld",(long)SysRunState.s_CPS.CPS2);
    buf[12] = SysRunState.s_CPS.State;

    SendData('P',(uint8_t*)buf,13);
}


/*******************************************************************************
功能：写报警参数命令(WF)响应
输入：unsigned char *cdata
输出：无
*******************************************************************************/
void ACK_CMD_B(u8 *cdata)
{
    SYS_ALARM wal;   
    memcpy((uint8_t*)&wal,cdata,sizeof(SYS_ALARM));

    SysRunState.stParam.s_Alarm.DoseAlarm = FloatToSmall(wal.DoseAlarm);
    SysRunState.stParam.s_Alarm.DosePreAlarm = FloatToSmall(wal.DosePreAlarm);
    SysRunState.stParam.s_Alarm.DoseRateAlarm = FloatToSmall(wal.DoseRateAlarm);
    SysRunState.stParam.s_Alarm.DoseRatePreAlarm = FloatToSmall(wal.DoseRatePreAlarm);

	SendData('B',NULL,0);
    SaveParam();

} 


/*******************************************************************************
功能：读报警参数命令(F)响应
输入：无
输出：无
*******************************************************************************/
void ACK_CMD_F(void)
{
    SYS_ALARM ral;
    ral.DoseAlarm = FloatToSmall(SysRunState.stParam.s_Alarm.DoseAlarm);
    ral.DosePreAlarm = FloatToSmall(SysRunState.stParam.s_Alarm.DosePreAlarm);
    ral.DoseRateAlarm = FloatToSmall(SysRunState.stParam.s_Alarm.DoseRateAlarm);
    ral.DoseRatePreAlarm = FloatToSmall(SysRunState.stParam.s_Alarm.DoseRatePreAlarm);

	SendData('F',(uint8_t*)&ral,sizeof(SYS_ALARM));
} 

/*******************************************************************************
功能：测温
输入：
输出：
*******************************************************************************/
void ACK_CMD_T(void)
{
    SendData('T',NULL,0);
}

/*******************************************************************************
功能：软件版本
输入：
输出：
*******************************************************************************/
void ACK_CMD_M(void)
{
    
     SendData('M',(uint8_t*)SysRunState.stParam.VerSion,6);
}

#if 0
/*******************************************************************************
功能：工作状态
输入：
输出：
*******************************************************************************/
void ACK_CMD_E(void)
{
	uint16_t state = 3;
	uint16_t m_state = WordToSmall(state);
	SendData('E',(uint8_t*)&m_state,2);
}

/*******************************************************************************
功能：清除累计剂量(1)响应
输入：
输出：
*******************************************************************************/
void ACK_CMD_ClearDoseSum(void)
{
	SysRunState.s_DoseMSG.Dose = 0;
    //SysRunState.s_DoseMSG.Dose_B = 0;
	SendData(1,NULL,0);
}

/*******************************************************************************
功能：清除最大剂量率(2)响应
输入：
输出：
*******************************************************************************/
void ACK_CMD_ClearMaxDoseRate(void)
{
	SysRunState.s_DoseMSG.MaxDoseRate = 0;
    //SysRunState.s_DoseMSG.MaxDoseRate_B = 0;
	SendData(2,NULL,0);
}

/*******************************************************************************
功能：探测器自检(3)响应
输入：
输出：
*******************************************************************************/
void ACK_CMD_SelfCheck(void)
{
	//自检
  	uint16_t result=1;
    uint16_t m_result;
	if(/*(Get_Low_Counter() < LOW_DIBENDI)||*/(Get_Low_Counter() > LOW_GAOBENDI)
	   /*||(Get_High_Counter() < HIGH_DIBENDI)*/||(Get_High_Counter() > HIGH_GAOBENDI))
	{
	  	result = 0;
	}
     m_result = WordToSmall(result); 
	SendData(3,(uint8_t*)&m_result,2);
}

/*******************************************************************************
功能：开关探测器(4)响应
输入：
输出：
*******************************************************************************/
void ACK_CMD_SensorONOFF(uint16_t state)
{
	SendData(4,NULL,0);
	//开关探测器
	if(state == 1)
	{
		SensorInit();
	}
	else
	{
	  	//DevSleep();
	  	SysRunState.NoUartTime = POWER_OFF_TIME+1;
	}
}

/*******************************************************************************
功能：电池电量查询(5)响应
输入：
输出：
*******************************************************************************/
STU_BATTERY s_Bat={0};
void ACK_CMD_Bat(void)
{
    STU_BATTERY bat;
    
	DeviceGetBatAlarm(&s_Bat);
    
	bat.Voltage = WordToSmall(s_Bat.Voltage);
    bat.Status = s_Bat.Status;
    bat.batPercent = s_Bat.batPercent; 
    
	SendData(5,(uint8_t*)&bat,sizeof(STU_BATTERY));
}

/*******************************************************************************
功能：报警确认(6)响应
输入：
输出：
*******************************************************************************/
void ACK_CMD_SureAlarm(void)
{
	//报警确认，关闭相应指示灯
	
	SendData(6,NULL,0);
}
#endif

/*******************************************************************************
功能：切换高低量程
输入：
输出：
*******************************************************************************/
void ACK_CMD_Q(unsigned char Gm)
{
	if (Gm == 1)
    {
        GDoseSeg = HIG_SEG;
	}
    else
    {
        GDoseSeg = LOW_SEG;
    }
    
	SendData(7,NULL,0);
}

//========================================================================
// 函数名称: WORD CheckSum(BYTE *buf, WORD len)
// 函数功能: 校验和函数
// 入口参数: @*buf：数据；@len：长度
// 函数返回: 校验结果
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================
WORD CheckSum(BYTE *buf, WORD len)
{
    WORD dwSum = 0;
    WORD i;

    for (i = 0; i < len; i++)
    {
        dwSum += buf[i];
    }
    return dwSum;
}
    
void SaveParam()
{
    EA = 0;//禁止中断
    WritePara();
    EA = 1;//允许中断
}

