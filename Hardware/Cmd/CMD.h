#ifndef CMD_H
#define CMD_H

#include "main.h"

#define WORKMODE1 0x0001
#define WORKMODE2 0x0002
#define WORKMODE3 0x0003

#define Sensor_Area 100//探测器面积

#define CMD_READ_NH    0xE1
#define CMD_WRITE_NH   0xE2
#define CMD_READ_FIX    0xE3
#define CMD_WRITE_FIX   0xE4


#pragma pack(1)
typedef struct
{
//	uint8_t head;	//
//	uint8_t cmd;
//	uint16_t length;
    uint8_t head;    //
    uint8_t length;
    uint8_t type;
    uint8_t channel;
    uint8_t cmd;
	//uint8_t *data;
	//uint16_t crcCheck;
}STU_CMD;
#pragma pack()

extern LP_SYSTEM_STTAE SysRunState;

DWORD DwordToSmall(DWORD dat);
float FloatToSmall(float dat);

void DataPro(uint8_t *cdata, uint8_t length);
void ACK_CMD_C(void);//握手命令
void ACK_CMD_H(uint8_t Address);//设置探测器地址

void ACK_CMD_R(void);//读参数命令
void ACK_CMD_W(uint8_t *cdata);//写参数命令
void ACK_CMD_N(unsigned char *cdata);//设置校准因子
void ACK_CMD_J(void);//读校准因子

void ACK_CMD_V(void);//读计数命令
void ACK_CMD_B(uint8_t *cdata);//写报警参数命令
void ACK_CMD_F(void);//读报警参数命令
void ACK_CMD_P(void);
void ACK_CMD_T(void);
void ACK_CMD_Q(unsigned char Gm);
void ACK_CMD_M(void);

void ReadCS();
void SetCS(u8 *cs);
void ReadFix();
void WriteFix(u8 *dat);

void GetPara(LP_PARAM *me);//初始化时，通过本函数将数据从info flash拷贝到RAM中
void WritePara();

uint8_t GetCautionStatus_Alphy(float CNT);
uint8_t GetCautionStatus_Beta(float CNT);
void Get_Cur_Effic(unsigned char netro_num);
WORD CheckSum(BYTE *buf,WORD len);
void Updata_BenDi_Value();
void SaveParam();

#endif

