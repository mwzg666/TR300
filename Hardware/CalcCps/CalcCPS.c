#include "Queue.h"
#include "main.h"
#include "CalcDoseRate.h"
#include "DoseRate.h"


#define CPS_AVG_BUG_SIZE (2)
#define CPS_COEFFICIENT (1)

unsigned long lCPS = 0;
unsigned long lCP100MS = 0;


const NH_PARAM NhParamL[NH_COUNT] = 
{
    //CPS       DoseRate
    {0,         0  },
    {1.72,      0.799},
    {17.05,     9.735},
    {145.97,    84.75},
    {792.66,    500},
    {1471.55,   1000},
    {5712.15,    8000},
        
    {10561.06,   10000}   // ����ʵ�ʲ���ʹ��
};

const NH_PARAM NhParamH[NH_COUNT] = 
{
    //CPS       DoseRate
    {0,         0  },
    {32.48,     1000},
    {228.51,    8000},
    {1091.09,   40710},
    {3064.53,   103900},
    {16332.33,  661100},
    {25483.0,   1178000},
    {42991.0,   2657000}
};


// ���ǵ��ֿ�����ͺ�������ͬʱ�������ֿ�ʹ�ù���buffer
//static squeue stFilterQ;
//static squeue stCPSQ;

//static squeue stFilterAlphaQ;
//static squeue stAlphaQ;

//static squeue stFilterBetaQ;
//static squeue stBetaQ;

// ƽ��buffer
//static squeue stAlphaCPSAvgQ,stBetaCPSAvgQ;

//static S_BENDI gs_AlphaBendi,gs_BetaBendi;

//static void Filter(squlink FilterQ, squlink CalcQ, unsigned long nNewCnt);
#ifdef ZGC_DOSE
static DataType SmoothCps(float parama, float paramb, float paramc, float RealCPS, squlink avgQ, float OlduSvh, S_BENDI *bendi, float *NewuSvh);


/******************************************************
Cps ����

void InitCalcCps(void)
{
    InitQueue(&stFilterQ, (FILTER_BUF_SIZE));
    InitQueue(&stCPSQ, (CPS_BUF_SIZE));
    InitQueue(&stCPSAvgQ, (CPS_AVG_BUG_SIZE));
}

// ÿһ�뱻����һ��
unsigned long CalcCps(void)
{
    DataType CPS = 0;
    //CPS = SmoothCps(&stCPSQ);
    return CPS;
}*/
/*
void EmtyCalcCpsQ(void)
{
    InitQueue(&stCPSQ, (CPS_BUF_SIZE));
    InitQueue(&stCPSAvgQ, (CPS_AVG_BUG_SIZE));
}

// ÿ100ms����һ��
void FilterCps(unsigned long nNewCnt)
{
    Filter(&stFilterQ, &stCPSQ, nNewCnt);
}

// ��ȡN���㣨N*100ms����������CPS���ж��Ƿ�
// ����CPS
unsigned long  CalcTrendCps(void)
{
    // 100ms * CALC_TREND_CPS_CNT
    unsigned long CPS = SumRear(&stCPSQ, CALC_TREND_CPS_CNT);
    // 1000ms
    float fCoEfficient = 1000.0f/(CALC_TREND_CPS_CNT*100.0F);
    CPS *= fCoEfficient;
    CPS *= CPS_COEFFICIENT;
    
    return CPS;
}
*/

/******************************************************
Low����
*/
void InitCalcLow(void)
{
    //InitQueue(&stFilterAlphaQ, (FILTER_BUF_SIZE));
    //InitQueue(&stAlphaQ, (CPS_BUF_SIZE));
    //InitQueue(&stAlphaCPSAvgQ, (CPS_AVG_BUG_SIZE-1));
}


void EmtyCalcLowQ(void)
{
    //InitQueue(&stAlphaQ, (CPS_BUF_SIZE));
    InitQueue(&stAlphaCPSAvgQ, (CPS_AVG_BUG_SIZE-1));
}

void EmtyBendi(void)
{
    memset((void*)&gs_AlphaBendi,0,sizeof(S_BENDI));
    memset((void*)&gs_BetaBendi,0,sizeof(S_BENDI));
	gs_AlphaBendi.OlduSvh2 = 0.5;
	gs_AlphaBendi.OlduSvh3 = 0.5;
	gs_AlphaBendi.OlduSvh4 = 0.5;
	gs_AlphaBendi.OlduSvh5 = 0.5;
	gs_AlphaBendi.OlduSvh6 = 0.5;
	gs_BetaBendi.OlduSvh2 = 0.5;
	gs_BetaBendi.OlduSvh3 = 0.5;
	gs_BetaBendi.OlduSvh4 = 0.5;
	gs_BetaBendi.OlduSvh5 = 0.5;
	gs_BetaBendi.OlduSvh6 = 0.5;
}
#endif

// ÿһ�뱻����һ��,�������������CPS������������
DataType CalcLow(float parama, float paramb, float paramc, float RealCPS, float OlduSvh, float *NewuSvh)
{
    DataType CPS = 0;
    #ifdef ZGC_DOSE
    CPS = SmoothCps(parama, paramb, paramc, RealCPS, &stAlphaCPSAvgQ, OlduSvh, &gs_AlphaBendi, NewuSvh);
    #else
    CpsFilter((DWORD)RealCPS);    
    CPS = CounterPH();
    if (CPS != -1)
    {
        #ifdef P2P_NH
        *NewuSvh = CpsToUsv_h(NhParamL, CPS);
        #else
        *NewuSvh = CpsToUsv_h(parama, paramb, paramc, CPS);
        #endif
    }
    #endif
    /*if(*NewuSvh > 1)
	{
	  	return *NewuSvh;
	}*/
    return CPS;
}

#ifdef ZGC_DOSE

// ÿ100ms����һ��
void FilterLow(unsigned long nNewCnt)
{
    //Filter(&stFilterAlphaQ, &stAlphaQ, nNewCnt);
}

/******************************************************
High����
*/
void InitCalcHigh(void)
{
    //InitQueue(&stFilterBetaQ, (FILTER_BUF_SIZE));
    //InitQueue(&stBetaQ, (CPS_BUF_SIZE));
    InitQueue(&stBetaCPSAvgQ, (CPS_AVG_BUG_SIZE-1));
}

void EmtyCalcHighQ(void)
{
    //InitQueue(&stBetaQ, (CPS_BUF_SIZE));
    InitQueue(&stBetaCPSAvgQ, (CPS_AVG_BUG_SIZE-1));
}
#endif

// ÿһ�뱻����һ��
DataType CalcHigh(float parama, float paramb, float paramc, float RealCPS, float OlduSvh, float *NewuSvh)
{
    DataType CPS = 0;
    #ifdef ZGC_DOSE
    CPS = SmoothCps(parama, paramb, paramc, RealCPS, &stBetaCPSAvgQ, OlduSvh, &gs_BetaBendi, NewuSvh);
    #else
    CpsFilter((DWORD)RealCPS);    
    CPS = CounterPH();
    if (CPS != -1)
    {
        #ifdef P2P_NH
        *NewuSvh = CpsToUsv_h(NhParamH, CPS);
        #else
        *NewuSvh = CpsToUsv_h(parama, paramb, paramc, CPS);
        #endif
    }
    #endif
    return CPS;
}


// ÿ100ms����һ��
void FilterHigh(unsigned long nNewCnt)
{
    //Filter(&stFilterBetaQ, &stBetaQ, nNewCnt);
}

//static float AllTimeCPS,AllTimeSec;//��ʱ���ۼӵļ�����ʱ��
/******************************************************
ƽ���㷨
//squlink Q ����
*/

#ifdef ZGC_DOSE

#define TURNINGUSV      		(10)//ת�ۼ����ʣ�usv
#define LOW_UPFAST      		(5)//�ͼ����仯����
#define HIGH_UPFAST     		(3)//�߼����仯����
#define LOW_UPDOWN_TIMES  		(2)//�ͼ�����Ӧʱ��
#define HIGHT_UPDOWN_TIMES  	(1)//����ͻ��ȡ��


DataType SmoothCps(float parama, float paramb, float paramc, float RealCPS, squlink avgQ, float OlduSvh, S_BENDI *bendi, float *NewuSvh)
{
    static DataType CPS = 0,ave=0;
    unsigned long Cnt;// = Lenqueue(Q);
    float uSvh,tempuSvh,realTimeCPSuSvh;
    float multiple;
	unsigned char updowntimes;


    if(OlduSvh < 0.5)
    {
        OlduSvh = 0.5;
    }
    bendi->OlduSvh6 = bendi->OlduSvh5;
    bendi->OlduSvh5 = bendi->OlduSvh4;
    bendi->OlduSvh4 = bendi->OlduSvh3;
    bendi->OlduSvh3 = bendi->OlduSvh2;
    bendi->OlduSvh2 = OlduSvh;
    //CPS = SumRear(Q, Cnt);
    CPS = RealCPS;//��ȡ����һ������

    EnterQueue(avgQ, CPS);//�ȴ���ƽ��ֵ���ݶ�����

    Cnt = Lenqueue(avgQ);

    ave = CPS;//SumRear(avgQ, 2)/2;

    //�ж��Ƿ�������
    uSvh = CpsToUsv_h(parama, paramb, paramc, ave);
	if(uSvh > 100000)//10m������ʵʱֵ
	{	
	  	*NewuSvh = uSvh;
		return ave; 
	}
    tempuSvh = uSvh;
    if(tempuSvh < 0.5)
    {
        tempuSvh = 0.5;
    }
    if((tempuSvh < TURNINGUSV)&&(bendi->OlduSvh3 < TURNINGUSV))//10u����5���仯
    {
        multiple = LOW_UPFAST;
		updowntimes = LOW_UPDOWN_TIMES;
    }
    else
    {
        multiple = HIGH_UPFAST;
		updowntimes = HIGHT_UPDOWN_TIMES;
    }
    if((tempuSvh > multiple*bendi->OlduSvh3)||(tempuSvh * multiple < bendi->OlduSvh3))
    {
        bendi->updownTimes++;
        if(bendi->updownTimes >= 2)
        {
            bendi->updownTimes = 0;
            if(Cnt >=updowntimes)
            {
                SetQLength(avgQ, updowntimes);
                Cnt = updowntimes;
            }
            ave = SumRear(avgQ, Cnt)/Cnt;//ǰCnt��ƽ��ֵ
            uSvh = CpsToUsv_h(parama, paramb, paramc, ave);
            *NewuSvh = uSvh;
			
            bendi->down30 = 0;
            bendi->startFlashFlag = 0;
            bendi->FlashTimes = 0;
            bendi->OlduSvh6 = uSvh;
            bendi->OlduSvh5 = uSvh;
            bendi->OlduSvh4 = uSvh;
            bendi->OlduSvh3 = uSvh;
            bendi->OlduSvh2 = uSvh;
			bendi->time = Cnt;
			bendi->cps = ave*Cnt;
            return ave;     
        }
    }
    else
    {
        bendi->updownTimes = 0;
    }

    if(Cnt >= 3)
    {
        ave = SumRear(avgQ, 3)/3;//ǰ3��ƽ��ֵ
        uSvh = CpsToUsv_h(parama, paramb, paramc, ave);
        if((uSvh > 1000.0)&&(abs(100.0*(uSvh-bendi->OlduSvh3)/(bendi->OlduSvh3+1.0)) > 6))//3���������1m�����Ҳ��������ϴ�ƽ����10%����ǰ3�������
        {
            SetQLength(avgQ, 3);
            *NewuSvh = uSvh;

			bendi->time = 3;
			bendi->cps = ave*3;

            bendi->OlduSvh6 = uSvh;
            bendi->OlduSvh5 = uSvh;
            bendi->OlduSvh4 = uSvh;
			bendi->OlduSvh3 = uSvh;
            bendi->OlduSvh2 = uSvh;
            return ave;
        }
    }

   
    if(Cnt >= 5)
    {
        realTimeCPSuSvh = CpsToUsv_h(parama, paramb, paramc, CPS);
        if(realTimeCPSuSvh < 0.5)
        {
            realTimeCPSuSvh = 0.5;
        }
        if(((100.0*(realTimeCPSuSvh-bendi->OlduSvh5)/(bendi->OlduSvh5+0.1)) > 50)/*&&(realTimeCPSuSvh >= 1)*/)//����
        {
            bendi->startFlashFlag = 1;
            if(bendi->down30 > 0)
            {
                bendi->down30--;
            }

        }
        else if(((100.0*(realTimeCPSuSvh-bendi->OlduSvh5)/(bendi->OlduSvh5+0.1)) > 28)&&(realTimeCPSuSvh >= 3))//����
        {
            bendi->startFlashFlag = 1;
            if(bendi->down30 > 0)
            {
                bendi->down30--;
            }
        }
        else if(((100.0*(realTimeCPSuSvh-bendi->OlduSvh5)/(bendi->OlduSvh5+0.1)) > 20)&&(realTimeCPSuSvh  >= 8))//����
        {
            bendi->startFlashFlag = 1;
            if(bendi->down30 > 0)
            {
                bendi->down30--;
            }
        }
        else if(((100.0*(bendi->OlduSvh5 - realTimeCPSuSvh)/(realTimeCPSuSvh+0.1)) > 50)/*&&(bendi->OlduSvh5 >= 1)*/)//�½�
        {
            bendi->down30++;
            bendi->startFlashFlag = 1;
        }
        else if(((100.0*(bendi->OlduSvh5 - realTimeCPSuSvh)/(realTimeCPSuSvh+0.1)) > 28)&&(bendi->OlduSvh5 >= 3))//�½�
        {
            bendi->startFlashFlag = 1;
            if(bendi->down30 > 0)
            {
                bendi->down30--;
            }
        }
        else if(((100.0*(bendi->OlduSvh5 - realTimeCPSuSvh)/(realTimeCPSuSvh+0.1)) > 20)&&(bendi->OlduSvh5 >= 8))//�½�
        {
            bendi->startFlashFlag = 1;
            if(bendi->down30 > 0)
            {
                bendi->down30--;
            }
        }
        else
        {
            if(bendi->down30 > 0)
            {
                bendi->down30--;
            }
            
        }
        if(bendi->down30 >= 9)
        {
            bendi->down30 = 0;
            if(bendi->OlduSvh5 < 1.5)
			{
			  	bendi->startFlashFlag = 0;
				bendi->FlashTimes = 0;
				
				SetQLength(avgQ, 5);
				ave = SumRear(avgQ, 5)/5;
				uSvh = CpsToUsv_h(parama, paramb, paramc, ave);
				*NewuSvh = uSvh;
				bendi->OlduSvh6 = uSvh;
				bendi->OlduSvh5 = uSvh;
				bendi->OlduSvh4 = uSvh;
				bendi->OlduSvh3 = uSvh;
				bendi->OlduSvh2 = uSvh;
				bendi->time = 5;
				bendi->cps = ave*5;
				return ave;
			}
        }
        
        if(bendi->startFlashFlag == 1)
        {
            bendi->FlashTimes++;
        }
        else
        {
            bendi->FlashTimes = 0;
        }
        if(bendi->FlashTimes >= 5)
        {
            ave = SumRear(avgQ, 5)/5;//ǰ5��ƽ��ֵ
            uSvh = CpsToUsv_h(parama, paramb, paramc, ave);
			tempuSvh = uSvh;
			if(tempuSvh < 0.5)
			{
			  	tempuSvh = 0.5;
			}
			if((((100*(tempuSvh-bendi->OlduSvh5)/(bendi->OlduSvh5+0.1) > 35))&&(bendi->OlduSvh5 > 10))//10u
                ||(((100*(bendi->OlduSvh5 - tempuSvh)/(bendi->OlduSvh5+0.1) > 35))&&(bendi->OlduSvh5 > 10)))//���������ϴ�ƽ����30%����ǰ5�������
            {
                SetQLength(avgQ, 5);
                *NewuSvh = uSvh;
                bendi->down30 = 0;
                bendi->OlduSvh6 = uSvh;
                bendi->OlduSvh5 = uSvh;
                bendi->OlduSvh4 = uSvh;
                bendi->OlduSvh3 = uSvh;
                bendi->OlduSvh2 = uSvh;
				
				bendi->startFlashFlag = 0;
            	bendi->FlashTimes = 0;
				bendi->time = 5;
				bendi->cps = ave*5;
                return ave;
            }
			Cnt = Lenqueue(avgQ);
			if(Cnt >= 10)
			{
			  	bendi->startFlashFlag = 0;
            	bendi->FlashTimes = 0;
				ave = SumRear(avgQ, 10)/10;//ǰ10��ƽ��ֵ
            	uSvh = CpsToUsv_h(parama, paramb, paramc, ave);
				tempuSvh = uSvh;
				if(tempuSvh < 0.5)
				{
					tempuSvh = 0.5;
				}
				if((((100*(tempuSvh-bendi->OlduSvh5)/(bendi->OlduSvh5+0.1) > 80)))//
                ||(((100*(bendi->OlduSvh5 - tempuSvh)/(bendi->OlduSvh5+0.1) > 80))))//���������ϴ�ƽ����30%����ǰ5�������
				{
					SetQLength(avgQ, 5);
					*NewuSvh = uSvh;
					bendi->down30 = 0;
					bendi->OlduSvh6 = uSvh;
					bendi->OlduSvh5 = uSvh;
					bendi->OlduSvh4 = uSvh;
					bendi->OlduSvh3 = uSvh;
					bendi->OlduSvh2 = uSvh;
					bendi->time = 5;
					bendi->cps = ave*5;
					return ave;
				}
				else if((((100*(tempuSvh-bendi->OlduSvh5)/(bendi->OlduSvh5+0.1) > 50))&&(bendi->OlduSvh5 > 1.5))//10u
                ||(((100*(bendi->OlduSvh5 - tempuSvh)/(bendi->OlduSvh5+0.1) > 50))&&(bendi->OlduSvh5 > 1.5)))//���������ϴ�ƽ����30%����ǰ5�������
				{
					SetQLength(avgQ, 5);
					*NewuSvh = uSvh;
					bendi->down30 = 0;
					bendi->OlduSvh6 = uSvh;
					bendi->OlduSvh5 = uSvh;
					bendi->OlduSvh4 = uSvh;
					bendi->OlduSvh3 = uSvh;
					bendi->OlduSvh2 = uSvh;
					bendi->time = 5;
					bendi->cps = ave*5;
					return ave;
				}
				else if((((100*(tempuSvh-bendi->OlduSvh5)/(bendi->OlduSvh5+0.1) > 35))&&(bendi->OlduSvh5 > 3))//10u
                ||(((100*(bendi->OlduSvh5 - tempuSvh)/(bendi->OlduSvh5+0.1) > 35))&&(bendi->OlduSvh5 > 3)))//���������ϴ�ƽ����30%����ǰ5�������
				{
					SetQLength(avgQ, 5);
					*NewuSvh = uSvh;
					bendi->down30 = 0;
					bendi->OlduSvh6 = uSvh;
					bendi->OlduSvh5 = uSvh;
					bendi->OlduSvh4 = uSvh;
					bendi->OlduSvh3 = uSvh;
					bendi->OlduSvh2 = uSvh;
					bendi->time = 5;
					bendi->cps = ave*5;
					return ave;
				}
				else if((((100*(tempuSvh-bendi->OlduSvh5)/(bendi->OlduSvh5+0.1) > 15))&&(bendi->OlduSvh5 > 100))//100u
                ||(((100*(bendi->OlduSvh5 - tempuSvh)/(bendi->OlduSvh5+0.1) > 15))&&(bendi->OlduSvh5 > 100)))//���������ϴ�ƽ����30%����ǰ5�������
				{
					SetQLength(avgQ, 5);
					*NewuSvh = uSvh;
					bendi->down30 = 0;
					bendi->OlduSvh6 = uSvh;
					bendi->OlduSvh5 = uSvh;
					bendi->OlduSvh4 = uSvh;
					bendi->OlduSvh3 = uSvh;
					bendi->OlduSvh2 = uSvh;
					bendi->time = 5;
					bendi->cps = ave*5;
					return ave;
				}
			}
        }
    }



    //ǰ�涼����������ʵ���������ݵ�ƽ��ֵ
	Cnt = Lenqueue(avgQ);
	//����3��ļ���������0.5��
	if((bendi->OlduSvh2 > 30)&&(bendi->OlduSvh3 > 30)&&(bendi->OlduSvh4 > 30))
	{
	  	if(Cnt > 60)
		{
		  	SetQLength(avgQ, 60);
			Cnt = 60;
		}
	  	ave = SumRear(avgQ, Cnt)/Cnt;//ǰCnt��ƽ��ֵ
		bendi->time += 1;
		bendi->cps += CPS;
	}
	else if((bendi->OlduSvh2 > 20)&&(bendi->OlduSvh3 > 20)&&(bendi->OlduSvh4 > 20))
	{
	  	if(Cnt > 90)
		{
		  	SetQLength(avgQ, 90);
			Cnt = 90;
		}
		ave = SumRear(avgQ, Cnt)/Cnt;//ǰCnt��ƽ��ֵ
		bendi->time += 1;
		bendi->cps += CPS;
	}
	else
	{
	  	ave = SumRear(avgQ, Cnt)/Cnt;//ǰCnt��ƽ��ֵ
		bendi->time += 1;
		bendi->cps += CPS;
	}
	/*
    else
	{
		//��ʱ���ۼ�ֵ
		bendi->time += 1;
		bendi->cps += CPS;
		ave = bendi->cps/bendi->time;
	}*/
    uSvh = CpsToUsv_h(parama, paramb, paramc, ave);
    *NewuSvh = uSvh;
	Cnt = Lenqueue(avgQ);
	if(Cnt > 15)
	{
	  	tempuSvh = SumRear(avgQ, 15)/15;
		uSvh = CpsToUsv_h(parama, paramb, paramc, tempuSvh);
	  	if((uSvh + 0.5 < *NewuSvh)&&(*NewuSvh < 1.5))
		{
		  	*NewuSvh = uSvh;
			ave = tempuSvh;
			bendi->cps = tempuSvh*15;
			bendi->time = 15;
			SetQLength(avgQ, 15);
		}
	}
    return ave;
}

#endif

