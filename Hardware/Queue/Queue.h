#ifndef QUEUE_H
#define QUEUE_H

#ifdef __cplusplus
extern  "C" {
#endif
  
#define MAX_SQUEUE_SIZE (128)

//������������
typedef  float  DataType;

typedef struct
{
    //���еĴ洢�ռ�
    //���е���󳤶�:120����ѭ�����У���ʹ��1��Ԫ�� 
    DataType cdata[MAX_SQUEUE_SIZE + 1];
    //��ͷ����βָ��
    int nFront;
    int nRear;
    // �û������ʹ��buffer�ĳ��ȣ���Ƭ���У�������ʹ��new��̬����
    unsigned int nSize;
}squeue,*squlink;

typedef struct
{
  	DataType cps;
    unsigned long time;
    unsigned long updownTimes;//ͻ�����
    //unsigned char up15;//15%�����仯����
    //unsigned char up25;//25%�����仯����
    //unsigned char up30;//30%�����仯����
    //unsigned char down15;//15%�����仯����
    //unsigned char down25;//25%�����仯����
    unsigned char down30;//30%�����仯����
    unsigned char startFlashFlag;//������־
    unsigned char FlashTimes;//�������ִ���
    DataType OlduSvh2;
    DataType OlduSvh3;
    DataType OlduSvh4;
    DataType OlduSvh5;
    DataType OlduSvh6;

}S_BENDI;


//��ʼ�����ÿնӣ���󳤶�Ϊ20��Ԫ�أ�������Ҫ�仯��󳤶�MAX_SQUEUE_SIZE��
void InitQueue(squlink Q, unsigned int nSize);

// FIFO �ӿ�
//����
int EnterQueue(squlink Q,DataType e);
//����
int DeQueue(squlink Q, DataType*  e);
//�ж϶����Ƿ�Ϊ�� 
int IsEmptyQueue(squlink Q);
//�ж϶����Ƿ���
int IsFullQueue(squlink Q);
// ����
int Lenqueue(squlink Q);

//ջ�ӿ�
//��ջ
int DeStack(squlink Q, DataType*  e);


// ��ͷ��ȡ��N������:1,2...n
DataType GetElement(squlink Q, int nOrder);
//����Ԫ��ֵ
int SetElement(squlink Q, int nOrder, DataType value);
// ǰN�����ܺͣ������ӣ�
DataType SumFront(squlink Q, int nCnt);
// ��N�����ܺͣ������ӣ�
DataType SumRear(squlink Q, int nCnt);

//���ö��г��ȣ��������¼�������
void SetQLength(squlink Q, int cnt);
#ifdef __cplusplus
}
#endif
#endif  