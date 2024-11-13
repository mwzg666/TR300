#include "Sensor.h"
#include "CalcDoseRate.h"
#include "CalcCPS.h"
#include "DoseRate.h"
#include "system.h"
#include "CMD.h"

u8 GDoseSeg = LOW_SEG;       //当前处在的段
u8 Dose_switch = 0;    //低量程段0：测量值为γ，1：测量值为β
u32 Low_CPS = 0;
u32 High_CPS = 0;
static float HightDoseRate;
static float LowDoseRate;

static float LowSmothCPS,HighSmothCPS;
static float LowSmothCPS2,HighSmothCPS2;

static float LowNOSmothCPS,HighNOSmothCPS;
static float LowSumCPS,HighSumCPS;

//β数据
static float HSumCPS;

static float SmothCPS_B;
static float NOSmothCPS_B;
static float SumCPS_B;

u32 InSenserCnt = 0;
float OldDr = 0.0;
float NewDr;
float RtCps,NewCps;
float CanshuA,CanshuB,CanshuC,CanshuD;
void SensorInit(void)
{
    //CanshuA = SysRunState.stParam.s_SysParam.DiYaCanshuA;
    //CanshuB = SysRunState.stParam.s_SysParam.DiYaCanshuB;
    //CanshuC = SysRunState.stParam.s_SysParam.DiYaCanshuC; 
    //CanshuD = 0.0;

    UseSLParam(0);
    memset((void*)&SysRunState.s_DoseMSG,0,sizeof(STU_DOSERATE));
}

void SensorMeasureBegin(void)
{ 
	Low_CPS = 0;
	High_CPS = 0;
	//GetCounter();
    //GetBataCounter();
}

void UseSLParam(float cps)
{
    if (cps <= 14.71)
    {
        CanshuA = 9.76996261670138E-15;
        CanshuB = 0.416404571866349;
        CanshuC = 0.00866370841409324;
        CanshuD = 0;
    }
    else if (cps <= 1220.8)
    {
        CanshuA  = 0.0167463070762324;
        CanshuB  = 0.541654614387292;
        CanshuC = 7.14371777215714E-05;
        CanshuD = 1.77334918007136E-08;
    }
    else if (cps <= 8561.06)
    {
        CanshuA  = -679.93211770926;
        CanshuB  = 1.24525771071468;
        CanshuC = -2.70261510263897E-05;
        CanshuD = 0;
    }
    else if (cps <= 11310.53)
    {
        CanshuA  = -211604.529633801;
        CanshuB  = 35.7075232732049;
        CanshuC = -0.00117461680924023;
        CanshuD = 0;
    }
    else
    {
       CanshuA  = -121190.506249211;
       CanshuB  = 14.4281926885134;
       CanshuC = 0;
       CanshuD = 0;
    }
    
}


void UseSHParam(float cps)
{
    if (cps < 2679.23)
    {
        CanshuA = -56.4956084021251;
        CanshuB = 33.1391426920277;
        CanshuC =  -0.00121629383397334;
        CanshuD = 0.0;
    }
    else
    {
        CanshuA = -9890.61062326864;
        CanshuB = 32.8126304817507;
        CanshuC = 0.000275557442490988;
        CanshuD = 0.0;
    }
}


float DrFix(float dr)
{
    float fix = 1.0;
    if (dr < 1.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[0]);
    }
    else if (dr < 10.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[1]);
    }
    else if (dr < 100.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[2]);
    }
    else if (dr < 1000.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[3]);
    }
    else if (dr < 10000.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[4]);
    }
    else if (dr < 100000.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[5]);
    }
    else if (dr < 1000000.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[6]);
    }
    else if (dr < 10000000.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[7]);
    }
    else if (dr < 100000000.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[8]);
    }
    else if (dr < 1.0E+9) //1000000000.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[9]);
    }
    else if (dr < 1.0E+10) // 10000000000.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[10]);
    }

    return (dr*fix);
}



void CaptureSensorPluseCounter(void)
{
	/**************测试用的****************************************/
	//Low_CPS = 10;
	//High_CPS = 10;
	/*****************************************************/
	
	//FilterLow(Low_CPS);
	//FilterHigh(High_CPS);
	//LowSumCPS += Low_CPS;
    float uSvh = 0;

	LowSumCPS = GetCounter();
	//HighSumCPS += High_CPS;
	HighSumCPS = GetHightCounter();

	if((LowSumCPS == 0)&&(SysRunState.LowChanneloff == 0))
	{
	  	SysRunState.LChannelNoCountTime++;
	}
	else
	{
	  	SysRunState.LChannelNoCountTime = 0;
	}
	if(HighSumCPS == 0)
	{
	  	SysRunState.HChannelNoCountTime++;
	}
	else
	{
	  	SysRunState.HChannelNoCountTime = 0;
	}
    switch(GDoseSeg)
    {
        case LOW_SEG:
        {
            SysRunState.LowChanneloff = 0;
            LowSmothCPS = CalcLow(
                        CanshuA, 
                        CanshuB, 
                        CanshuC,
                        LowSumCPS, 
                        LowDoseRate,
                        &uSvh);
            if (LowSmothCPS != -1)
            {
                uSvh *= SysRunState.stParam.s_SysParam.Canshu1;
                LowDoseRate = uSvh;
            }
            //SysRunState.stParam.s_SysParam.Canshu1 = SysRunState.stParam.s_Jiaozhun.DI_C;
            UseSLParam(LowSmothCPS);
            SysRunState.s_DoseMSG.DoseRate = LowDoseRate;

            if (SysRunState.s_DoseMSG.DoseRate >= USE_LOW_USV)// &&
                  //(SysRunState.s_DoseMSG.C2 >= 30)  ) // 确保是伽马才切换
            {
                GDoseSeg = HIG_SEG;
                ClearCounter();
            }
            break;
        }

        case HIG_SEG:
        {
            SysRunState.LowChanneloff = 1;
            HighSmothCPS = CalcHigh(
                      CanshuA, 
                      CanshuB, 
                      CanshuC,
                      HighSumCPS, 
                      HightDoseRate,
                      &uSvh);
            if (HighSmothCPS != -1)
            {
                uSvh *= SysRunState.stParam.s_SysParam.Canshu2;
                HightDoseRate = uSvh;
            }
            //SysRunState.stParam.s_SysParam.Canshu2 = SysRunState.stParam.s_Jiaozhun.GAO_C;
            UseSHParam(HighSmothCPS);
            SysRunState.s_DoseMSG.DoseRate = HightDoseRate;

            if(SysRunState.s_DoseMSG.DoseRate < USE_HIGH_USV)
            {
                GDoseSeg = LOW_SEG;
                ClearCounter();
            }
            break;
        }
        default: GDoseSeg = LOW_SEG;break;
        
    }
                 
	LowNOSmothCPS = LowSumCPS;
	HighNOSmothCPS = HighSumCPS;	
		
	HighSumCPS = 0;
	LowSumCPS = 0;
	
	/*if(SysRunState.s_DoseMSG.C1 > 1)
	{
		//剂量率大于1，停止
		LowSumCPS = 0;
	}*/
	
	SysRunState.s_CPS.CPS1 = LowNOSmothCPS;
	SysRunState.s_CPS.CPS2 = HighNOSmothCPS;

    SysRunState.s_DoseMSG.DoseRate = DrFix(SysRunState.s_DoseMSG.DoseRate);
	SysRunState.s_DoseMSG.Dose += SysRunState.s_DoseMSG.DoseRate/3600.0f;
	
//	if(SysRunState.s_DoseMSG.DoseRate>SysRunState.s_DoseMSG.MaxDoseRate)
//	{
//		SysRunState.s_DoseMSG.MaxDoseRate = SysRunState.s_DoseMSG.DoseRate;
//	}
    
	CalcAlarmState(&SysRunState);           

}

float Get_Low_Counter(void)
{
	return LowNOSmothCPS;
}

float Get_High_Counter(void)
{
	return HighNOSmothCPS;
}

float Get_Low_Smooth_Counter(void)
{
	return LowSmothCPS;
}

float Get_High_Smooth_Counter(void)
{
	return HighSmothCPS;
}
#if 0
u16 CalcAlarmState(LP_SYSTEM_STTAE *me)
{
#if 0
  	/* 剂量当量报警检查 */	
	if ((me->s_DoseMSG.Dose >= me->stParam.s_Alarm.DoseAlarm)&&(me->stParam.s_Alarm.DoseAlarm > 0)) 
	{ 
		me->s_DoseMSG.DoSt = 2;
    } 
	/* 剂量当量预警检查 */	
	else if((me->s_DoseMSG.Dose >= me->stParam.s_Alarm.DosePreAlarm)&&(me->stParam.s_Alarm.DosePreAlarm > 0)) 
	{ 
		me->s_DoseMSG.DoSt = 1;
    } 
#endif
	
	//U16 alarmState = me->Alarmstate&BATTARY_LOW_BIT;
  	if(me->s_DoseMSG.DoseRate >= 9999999)//10Sv以上则是过载报警
	{
	  	me->s_DoseMSG.DoseRate = 9999999;
		me->s_DoseMSG.State = 5;
		return true;
	}
	
	/* 剂量当量率报警检查 */	
	if ((me->s_DoseMSG.DoseRate >= me->stParam.s_Alarm.DoseRateAlarm)&&(me->stParam.s_Alarm.DoseRateAlarm > 0)) 
	{ 
		if((++me->DoseRateAlarmCnt) >= MIB_CST_DOSERATE_THRESHOLD_ALARM) {//连续两次报警则认为报警
			me->s_DoseMSG.State = 2;
			return true;
		}
    } else {
		me->DoseRateAlarmCnt= 0x0;
		me->s_DoseMSG.State = 0;
	}
	
	/* 剂量当量率预警检查 */	
	if ((me->s_DoseMSG.DoseRate >= me->stParam.s_Alarm.DoseRatePreAlarm)&&(me->s_DoseMSG.DoseRate < me->stParam.s_Alarm.DoseRateAlarm))
	{ 
		if((++me->DoseRatePreAlarmCnt) >= MIB_CST_DOSERATE_THRESHOLD_WARNING) {//连续两次报警则认为报警
			me->s_DoseMSG.State = 1;
			return true;
		}
    } else {
		me->DoseRatePreAlarmCnt= 0x0;
		me->s_DoseMSG.State = 0;
	}
	
	//if((SysRunState.LChannelNoCountTime>60)&&(SysRunState.HChannelNoCountTime>1200))//低通道1分钟无数据，高通道10分钟无数据则报双探测器异常
	//{
	//  	me->s_DoseMSG.DRSt = 8;
	//}
	//else 
    if(SysRunState.LChannelNoCountTime>60)//低通道1分钟无数据,探测器异常
	{
	  	me->s_DoseMSG.State = 6;
	}
	//else if(SysRunState.HChannelNoCountTime>1200)//高通道20分钟无数据,探测器异常
	//{
	  	//me->s_DoseMSG.DRSt = 7;
	//}
	else
	{
	  	me->s_DoseMSG.State = 0;
	}
	return true;
}
#endif

u16 CalcAlarmState(LP_SYSTEM_STTAE *me)
{
    float ov;
    
    #if 1
    /* 剂量当量报警检查 */    
    if ((me->s_DoseMSG.Dose >= me->stParam.s_Alarm.DoseAlarm)
        &&(me->stParam.s_Alarm.DoseAlarm > 0)) 
    { 
        me->s_DoseMSG.State |= DOSE_ALARM_BIT;
    } 
    else
    {
        me->s_DoseMSG.State &= ~DOSE_ALARM_BIT;
    }
    
    /* 剂量当量预警检查 */    
    if( (me->s_DoseMSG.Dose >= me->stParam.s_Alarm.DosePreAlarm)
        &&(me->stParam.s_Alarm.DosePreAlarm > 0)
        &&(me->s_DoseMSG.Dose < me->stParam.s_Alarm.DoseAlarm) ) 
    { 
        me->s_DoseMSG.State |= DOSE_PRE_ALARM_BIT;
    } 
    else
    {
        me->s_DoseMSG.State &= ~DOSE_PRE_ALARM_BIT;
    }
    #endif    
    
    /* 剂量当量率报警检查 */    
    if ( (me->s_DoseMSG.DoseRate >= me->stParam.s_Alarm.DoseRateAlarm)
        &&(me->stParam.s_Alarm.DoseRateAlarm > 0) ) 
    { 
        if((++me->DoseRateAlarmCnt) >= MIB_CST_DOSERATE_THRESHOLD_ALARM) 
        {   
            //连续两次报警则认为报警
            me->s_DoseMSG.State |= DOSERATE_ALARM_BIT;
        }
    } 
    else 
    {
        me->DoseRateAlarmCnt= 0x0;
        me->s_DoseMSG.State &= ~DOSERATE_ALARM_BIT;
    }
    
    /* 剂量当量率预警检查 */    
    if ( (me->s_DoseMSG.DoseRate >= me->stParam.s_Alarm.DoseRatePreAlarm)
        &&(me->s_DoseMSG.DoseRate < me->stParam.s_Alarm.DoseRateAlarm) )
    { 
        if((++me->DoseRatePreAlarmCnt) >= MIB_CST_DOSERATE_THRESHOLD_WARNING) 
        {
            //连续两次报警则认为报警
            me->s_DoseMSG.State |= DOSERATE_PRE_ALARM_BIT;
        }
    } 
    else 
    {
        me->DoseRatePreAlarmCnt= 0x0;
        me->s_DoseMSG.State &= ~DOSERATE_PRE_ALARM_BIT;
    }

    
    // 过载
    if (GDoseSeg == LOW_SEG) 
    {
        // 单GM管
        ov = 200.0*1000;
    }
    else
    {
        ov = 10.0*1000*1000;
    }
    
    if ((me->s_DoseMSG.Dose >= 10.0*1000*1000)||(me->s_DoseMSG.DoseRate >= ov))
    {
        me->s_DoseMSG.State |= OVER_RANGE_BIT;
        me->s_DoseMSG.DoseRate = ov;
    }
    else
    {
        me->s_DoseMSG.State &= ~OVER_RANGE_BIT;
    }
    
    if(((SysRunState.LChannelNoCountTime > 60)&&(GDoseSeg == LOW_SEG))
      ||(SysRunState.LChannelNoCountTime > 600))//1分钟无数据,探测器异常
    {
        me->s_DoseMSG.State |= LOW_BACK_BIT;
    }
    else
    {
        me->s_DoseMSG.State &= ~LOW_BACK_BIT;
    }
    
    if(me->s_CPS.State != me->s_DoseMSG.State)//报警信息发生变化则，重启蜂鸣器报警
    {
        SysRunState.keyValue = 0;
    }
    
    me->s_CPS.State = me->s_DoseMSG.State;
    return true;
}



