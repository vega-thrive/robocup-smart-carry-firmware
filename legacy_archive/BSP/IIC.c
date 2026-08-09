#include "main.h"
#include "gpio.h"
#include "IIC.h"

extern volatile uint8_t Angle_I2C[2];


void delayiic(u32 m)
{
//	m*=90;
	m*=160;	
	while(m--);
}
//��ʼ��IIC
void IIC_Init(void)
{
	IIC_SCL=1;
	IIC_SDA=1;
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delayiic(2);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delayiic(2);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delayiic(1);
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	delayiic(1);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA=1;delayiic(1);	   
	IIC_SCL=1;delayiic(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delayiic(1);
	IIC_SCL=1;
	delayiic(1);
	IIC_SCL=0;
}
//������ACKӦ��
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delayiic(1);
	IIC_SCL=1;
	delayiic(1);
	IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delayiic(1);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		delayiic(1); 
		IIC_SCL=0;	
		delayiic(1);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delayiic(1);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delayiic(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}
//������
u8 IIC_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 2;
	}
//    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
	IIC_Send_Byte((addr<<1)|1);//����������ַ+������	
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//������,����nACK 
		else *buf=IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++;
	}
    IIC_Stop();	//����һ��ֹͣ���� 
	return 0;	
}
u8 IIC_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
 	IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//����������ַ+д����
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();		//�ȴ�Ӧ��
	
	for(int i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);
		IIC_Wait_Ack();		//�ȴ�Ӧ��		
	}
	IIC_Stop();
	
	return 1;
}

