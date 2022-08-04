/**
  ******************************************************************************
  * @file    universal.c
  * @author  ������ҵ��ѧ-��ͩ
  * @version V1.0
  * @date    2018/11/8
  * @brief   ͨ�ú���
  ******************************************************************************
  * @attention
  * �������д��ͨ�ú����������Ժ����������ĳ����ϣ�ֱ�ӿ�����OK����
  *	2019��3��12�ս��и��£�ԲȦ��������������BUG�޸������ٽ�ֵ�������������޸������Ǵ������������������޸ġ�
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"

/**
  * @brief  б�º��������ڲ���ʽPID
  * @param  input rev step
  * @retval ֱ��ͨ��ָ���޸���������ֵ
  */
void Func_Ramp(s16 input,s16 *rev,s16 step)
{
	switch(Func_Signal(input - *rev))
	{
		case 1:
			if(input - *rev > step)
				*rev += step;
			else
				*rev = input;
			break;
		case -1:
			if(*rev - input > step)
				*rev -= step;
			else
				*rev = input;		
			break;
		default:
			break;
	}
}

/**
  * @brief  ������б��
  * @param  input rev step
  * @retval ֱ��ͨ��ָ���޸���������ֵ
  */
void Func_FRamp(s16 input,float *rev,float step)
{
	switch(Func_Signal(input - *rev))
	{
		case 1:
			if(input - *rev > step)
				*rev += step;
			else
				*rev = input;
			break;
		case -1:
			if(*rev - input > step)
				*rev -= step;
			else
				*rev = input;		
			break;
		default:
			break;
	}
}

/**
  * @brief  ������б�º��������ڲ���ʽPID
  * @param  input rev step
  * @retval ֱ��ͨ��ָ���޸���������ֵ
  */

void Func_CircleRamp(s16 input,s16 *rev,s16 step)
{
	if(Func_Abs(input-*rev) > 4096)
	{
		switch(Func_Signal(input - *rev))
		{
			case 1:
				if(8192 + *rev -input > step)
					*rev -= step;
				else
					*rev = input;
				break;
			case -1:
				if(8192 + input - *rev > step)
					*rev += step;
				else
					*rev = input;		
				break;
			default:
				break;
		}
	}
	else
	{
		switch(Func_Signal(input - *rev))
		{
			case 1:
				if(input - *rev > step)
					*rev += step;
				else
					*rev = input;
				break;
			case -1:
				if(*rev - input > step)
					*rev -= step;
				else
					*rev = input;		
				break;
			default:
				break;
		}
	}
//	*rev = Func_ValueRannge(*rev,8192,0);
}
/**
  * @brief  ��ȡ��ֵ������
  * @param  value
  * @retval ����
  */
s8 Func_Signal(s16 value)
{
	if(value < 0)
		return -1;
	else
		return 1;
}

/**
  * @brief  ��ȡ��ֵ�ľ���ֵ
  * @param  value
  * @retval ����ֵ
  */
float Func_Abs(float value)
{
	if(value >= 0)
		return value;
	else 
		return -value;	
}

/**
  * @brief  �Ա�������ֵ�����޷�
  * @param  value ����ֵ
  * @param  mode  1����Ȧģʽ����ֵ�����ֵ����Сֵ֮�⣩ 0����Ȧģʽ����ֵ����Сֵ�����ֵ֮�䣩
  * @retval �޷�ֵ
  */
float Func_Ramp_Limit(float value,float max,float min,u8 mode)
{
//	value = Func_ValueRannge(value,8192,0);
	if(mode)
	{
		if(value < min && value > max)
		{
			if(value >  ((max + min) / 2))
				return min;
			else
				return max;
		}
	}
	else
	{
		if(((max + min) / 2) > 4096)
		{
			if((value < min) && (value > ((max + min) / 2 - 4096)))
				return min;
			else if(value < ((max + min) / 2 - 4096) || value > max)
				return max;
		}
		else
		{
			if((value > max) && (value < ((max + min) / 2 + 4096)))
				return max;
			else if(value > ((max + min) / 2 + 4096) || value < min)
				return min;			
		}
	}
	return value;	
}

/**
  * @brief  ����ֵ�����޷�
  * @param  value
  * @retval �޷�ֵ
  */
float Func_Limit(float value,float max,float min)
{
	if(value>max)
		return value=max;
	else if(value<min)
		return value=min;
	else
		return value;
} 

/**
  * @brief  ����ֵ��Χ���д���
  * @param  value
  * @retval 
  */
s16 Func_ValueRannge(s16 value,s16 max,s16 min)
{
	if(value>max)
		value = value % max;
	else if(value<min)
		value = value - min + max;
	return value;
}

//�Ľװ�����˹�˲���
const float b[5]={0.0009334986129548445,0.003733994451819,0.005600991677729,0.003733994451819,0.0009334986129548445};
const float a[5]={1,-2.976844333696732,3.422309529377639,-1.786106600218039,0.355577382344410};

float xBuf[5]={0};
float yBuf[5]={0};

float IIRLowPass(float x)
{
	int i;
	//����֮ǰBuf��ǰ�ƶ�һ��λ�ã��Ա���֮ǰBuf�����ݣ�
	for(i=4; i>0;i--)
	{
		yBuf[i] = yBuf[i-1];
		xBuf[i] = xBuf[i-1];
	}
	xBuf[0] = x;
	yBuf[0] = 0;
	for(i=1;i<5;i++)
	{
		yBuf[0] = yBuf[0] + b[i]*xBuf[i];
		yBuf[0] = yBuf[0] - a[i]*yBuf[i];
	}
	yBuf[0] = yBuf[0] + b[0]*xBuf[0];
	return yBuf[0];
}
