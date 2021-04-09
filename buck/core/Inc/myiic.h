#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 

//IIC 驱动代码	   
//STM32F4工程模板-库函数版本
//淘宝店铺：http://mcudev.taobao.com									  
////////////////////////////////////////////////////////////////////////////////// 	
#include "stm32f4xx.h"
/*
	PB10 :  I2C_SCL
	PB11 :  I2C_SDA
	I2C2
*/
#ifndef u8
#define uint8_t unsigned char 
#define u8 uint8_t

#endif

//IO方向设置
#define SDA_IN  {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=0<<11*2;}	//PB11输入模式
#define SDA_OUT {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=1<<11*2;} //PB11输出模式
//IO操作函数	 
#define IIC_SCL    PBout(10) //SCL
#define IIC_SDA    PBout(11) //SDA	 
#define READ_SDA   PBin(11)  //输入SDA 

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  
#endif
















