//#include <msp430x21x2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Queue.h"

#if 0
//�ÿն�
void InitQueue(squlink Q, unsigned int nSize)
{
    if (!Q)
    {
        return;
    }
    
    Q->nFront = 0;
    Q->nRear = 0;
    //���е���󳤶�:ѭ�����У���ʹ��1��Ԫ�� 
    if (nSize > MAX_SQUEUE_SIZE)
    {
         nSize = MAX_SQUEUE_SIZE ;
    }
    Q->nSize = nSize+1;
}

//�ж϶����Ƿ�Ϊ�� 
int IsEmptyQueue(squlink Q)
{
    int bRet = 0;
     
    if (!Q)
    {
        return 1;
    }
    
    if(Q->nFront == Q->nRear)
    {
        bRet = 1;
    }
    else
    {
        bRet = 0;
    }
    
    return bRet;
}

int IsFullQueue(squlink Q)
{
    int bRet = 0;
     
    if (!Q)
    {
        return 1;
    }
    
    if(((Q->nRear + 1) % Q->nSize) == Q->nFront)
    {
        bRet = 1;
    }
    else
    {
        bRet = 0;
    }
    
    return bRet;
}

//����
int EnterQueue(squlink Q,DataType e)
{
    if (!Q)
    {
        return 0;
    }
    
    //����,�򽫶�ָ�����һλ
    if(((Q->nRear + 1) % Q->nSize) == Q->nFront)
    {
        Q->nFront = (Q->nFront + 1) % Q->nSize;
    }
    
    Q->data[Q->nRear] = e;
    Q->nRear = (Q->nRear + 1) % Q->nSize;
   
    return 1;
}

//����
int DeQueue(squlink Q, DataType*  e)
{
    if (!Q)
    {
        return 0;
    }
    
    int bRet = 0;
    //�ӿ�
    if(IsEmptyQueue(Q))
    {
        bRet = 0;
    }
    else
    {
        *e = Q->data[Q->nFront];
        Q->nFront = (Q->nFront+1) % Q->nSize;
        bRet = 1;
    }
    
    return bRet;
}

//��Q�е�ǰԪ�ظ���
int Lenqueue(squlink Q)
{
    int i = 0;
    if (!Q)
    {
        return 0;
    }
    
    i = (Q->nRear - Q->nFront + Q->nSize) % Q->nSize;
    
    return(i);
}


//ջ�ӿ�
//��ջ
int DeStack(squlink Q, DataType*  e)
{
    if (!Q)
    {
        return 0;
    }
    
    int bRet = 0;
    //�ӿ�
    if(IsEmptyQueue(Q))
    {
        bRet = 0;
    }
    else
    {
        Q->nRear = (Q->nRear - 1 + Q->nSize) % Q->nSize;
        *e = Q->data[Q->nRear];
        bRet = 1;
    }
    
    return bRet;
}


// ǰN�����ܺͣ������ӣ�
DataType SumFront(squlink Q, int nCnt)
{
    if (!Q || Lenqueue(Q) < nCnt)
    {
        return 0;
    }
    DataType nSum = 0;
    for(int i = 0; i < nCnt; i++)
    {
        nSum += Q->data[((Q->nFront+i) % Q->nSize)];
    }
    
    return nSum;
}

// ��N�����ܺͣ������ӣ�
DataType SumRear(squlink Q, int nCnt)
{
    if (!Q || Lenqueue(Q) < nCnt)
    {
        return 0;
    }
    DataType nSum = 0;
    for(int i = 0; i < nCnt; i++)
    {
        nSum += Q->data[((Q->nRear - 1 - i + Q->nSize) % Q->nSize)];
    }
    
    return nSum;
}


// ��ͷ��ȡ��N������:1,2...n
DataType GetElement(squlink Q, int nOrder)
{
    if (!Q || Lenqueue(Q) <= 0)
    {
        return 0;
    }
    
    DataType nRet = 0;
    nRet = Q->data[((Q->nFront + nOrder - 1) % Q->nSize)];
    return nRet;
}
//���ö��г���
void SetQLength(squlink Q, int cnt)
{
  	Q->nFront = (Q->nSize + Q->nRear - cnt)% Q->nSize;
}

// ����
int SetElement(squlink Q, int nOrder, DataType value)
{
    if (!Q)
    {
        return 0;
    }
    
    int index = (Q->nFront + nOrder - 1) % Q->nSize;
    if (index == Q->nRear)
    {
      return 0;
    }
    Q->data[index] = value;
    return 1;
}
#endif

