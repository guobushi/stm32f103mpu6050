#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "./systick/bsp_SysTick.h"
#include "./led/bsp_led.h"
#include "./usart/bsp_usart.h"
#include "./mpu6050/mpu6050.h"
#include "./i2c/bsp_i2c.h"
#include "bsp_dht11.h"
#include "core_delay.h"


//设置是否使用LCD进行显示，不需要的话把这个宏注释掉即可




#define USE_LCD_DISPLAY
#ifdef USE_LCD_DISPLAY
#include "./lcd/bsp_ili9341_lcd.h"
#endif

#define X_AXLE  0
#define Y_AXLE  1
#define Z_AXLE  2
#define DATA_COMPARE 21000 

/* MPU6050数据 */
short Accel[3];
short Gyro[3];
float Temp;

short data_temp[60][60];
int step_count=0,step_count1;

static void MPU6050_DataAnalyze(short hold_value)
{
	
    int i=0;
    
        for(i=0;i<50-2;i++)
            {
                //X轴活跃                 
                if((data_temp[i][X_AXLE]>=data_temp[i][Y_AXLE])&&(data_temp[i][X_AXLE]>=data_temp[i][Z_AXLE]))
                {
                    //计算斜率，连续判断3次值，排除误差
                    if((data_temp[i][X_AXLE]-hold_value)<0) 
                        if((data_temp[i+1][X_AXLE]-hold_value)<0)
                            if((data_temp[i+2][X_AXLE]-hold_value)<0)
                        {
																	printf("data_tempY:%d\n",data_temp[i][X_AXLE]);
                                  if(data_temp[i][X_AXLE] > DATA_COMPARE)
                                    step_count++;
                        }
                }
                //Y轴活跃
                else  if((data_temp[i][Y_AXLE]>=data_temp[i][X_AXLE])&&(data_temp[i][Y_AXLE]>=data_temp[i][Z_AXLE]))
                {
                    //连续判断3次值，排除误差
                    if((data_temp[i][X_AXLE]-hold_value)<0)
                        if((data_temp[i+1][X_AXLE]-hold_value)<0)
                            if((data_temp[i+2][X_AXLE]-hold_value)<0)
                        {             
																		printf("data_tempY:%d\n",data_temp[i][Y_AXLE]);
                                    if(data_temp[i][Y_AXLE] > DATA_COMPARE)
                                    step_count++;                        
                        }
                }
                //Z轴活跃
                else if((data_temp[i][Z_AXLE]>=data_temp[i][Y_AXLE])&&(data_temp[i][Z_AXLE]>=data_temp[i][X_AXLE]))
                {
                  //连续判断3次值，排除误差
                    if((data_temp[i][X_AXLE]-hold_value)<0) 
                        if((data_temp[i+1][X_AXLE]-hold_value)<0)
                            if((data_temp[i+2][X_AXLE]-hold_value)<0)
                        {
																		printf("data_tempZ:%d\n",data_temp[i][Z_AXLE]);
                                    if(data_temp[i][Z_AXLE] > DATA_COMPARE)
                                    step_count++;
                        }
                }
    
            }
}

static void MPU6050_DateConver(short *data_src)
{
	
  static int i_save= 0;
    int i=0,j=0,k;
    short max,min,hold_value;
		for(k=0;k<50;k++){
			MPU6050ReadAcc(Accel);
			data_temp[i_save][0] = Accel[0];
			data_temp[i_save][1] = Accel[1];
			data_temp[i_save][2] = Accel[2];
		
			//保存采样值
			if ((++i_save)==50) 
			{
					//找出最大值和最小值
					max=min=data_temp[0][0];
					for(i=0;i<50;i++)
							for(j=0;j<3;j++)
							{
									if(max<data_temp[i][j]) max=data_temp[i][j];
									if(min>data_temp[i][j]) min=data_temp[i][j];
							}
							//计算动态阙值
							hold_value = (max+min)/2;
							//计算步数
						MPU6050_DataAnalyze(hold_value);
					//重新采样
					i_save=0;
			}
	}
		
}




/**
  * @brief  主函数
  * @param  无  
  * @retval 无
  */
int main(void)
{
  char dispBuff[100];
  int k1;
	DHT11_Data_TypeDef DHT11_Data;
#ifdef USE_LCD_DISPLAY
  char cStr [ 70 ];
	ILI9341_Init ();         //LCD 初始化
  ILI9341_GramScan ( 6 );
	LCD_SetFont(&Font8x16);
	LCD_SetColors(RED,BLACK);
	ILI9341_Clear ( 0, 0, 240, 320);	
  ILI9341_DispString_EN ( 20, 20, "This is a MPU6050 demo" );
#endif
  
  //初始化systick
	SysTick_Init();
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;
	/* LED 端口初始化 */
	LED_GPIO_Config();
	/* 串口1通信初始化 */
	USART_Config();
  /*DHT11初始化*/
	DHT11_Init ();
	//I2C初始化
	i2c_GPIO_Config();
  //MPU6050初始化
	MPU6050_Init();
  if (MPU6050ReadID() == 0)
  {
    printf("\r\n没有检测到MPU6050传感器！\r\n");
    LED_RED; 
    #ifdef USE_LCD_DISPLAY			
      /*设置字体颜色及字体的背景颜色*/
      LCD_SetColors(BLUE,BLACK);	
      ILI9341_DispStringLine_EN(LINE(4),"No MPU6050 detected! ");			
      ILI9341_DispStringLine_EN(LINE(5),"Please check the hardware connection! ");		
    #endif
		while(1);	
	}
	


  
  /* 配置SysTick定时器和中断 */
  SysTick_Init(); //配置 SysTick 为 1ms 中断一次，在中断里读取传感器数据
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //启动定时器
  
  
  while(1)
  {
//    if( task_readdata_finish ) //task_readdata_finish = 1 表示读取MPU6050数据完成
//    {
//    
//      printf("\r\n加速度： %8d%8d%8d    ",Accel[0],Accel[1],Accel[2]);
//      printf("陀螺仪： %8d%8d%8d    ",Gyro[0],Gyro[1],Gyro[2]);
//      printf("温度： %8.2f",Temp);
//			
//			
//				
//      #ifdef USE_LCD_DISPLAY	
//      ILI9341_DispStringLine_EN(LINE(7),"Acceleration");		
//      sprintf ( cStr, "%8d%8d%8d",Accel[0],Accel[1],Accel[2] );	//加速度原始数据
//      ILI9341_DispStringLine_EN(LINE(8),cStr);					

//      ILI9341_DispStringLine_EN(LINE(11),"Gyro        :");			
//      sprintf ( cStr, "%8d%8d%8d",Gyro[0],Gyro[1],Gyro[2] );	//角原始数据
//      ILI9341_DispStringLine_EN(LINE(12),cStr);	
//			
//      sprintf ( cStr, "Temperture  :%8.2f",Temp );	//温度值
//      ILI9341_DispStringLine_EN(LINE(15),cStr);

//      #endif
//      
//      task_readdata_finish = 0; // 清零标志位
//    }
//		  Delay_ms(2000);
			/*调用DHT11_Read_TempAndHumidity读取温湿度，若成功则输出该信息*/
			if( DHT11_Read_TempAndHumidity ( & DHT11_Data ) == SUCCESS)
			{
        ILI9341_DispStringLine_EN(LINE(0),"YH DHT11 test");
        
        /* 显示温度 */
        sprintf(dispBuff,"Temperature : %d.%d ",DHT11_Data.temp_int, DHT11_Data.temp_deci);
        LCD_ClearLine(LINE(1));	/* 清除单行文字 */
        ILI9341_DispStringLine_EN(LINE(1),dispBuff);
        
        /* 显示湿度 */
        sprintf(dispBuff,"Humidity : %d.%d%% ",DHT11_Data.humi_int, DHT11_Data.humi_deci);
        LCD_ClearLine(LINE(2));	/* 清除单行文字 */
        ILI9341_DispStringLine_EN(LINE(2),dispBuff);
			}			
			else
			{
        LCD_ClearLine(LINE(1));	/* 清除单行文字 */
        LCD_ClearLine(LINE(2));	/* 清除单行文字 */
				ILI9341_DispStringLine_EN(LINE(1),"Read DHT11 ERROR");
        ILI9341_DispStringLine_EN(LINE(2),"Read DHT11 ERROR");
			}
			
			
//			MPU6050_DateConver(Accel);
//			step_count1=step_count/48;
//			sprintf ( cStr, "step_count  :%d",step_count1 );	
//      ILI9341_DispStringLine_EN(LINE(16),cStr);
			
		 for(k1=0;k1<50;k1++){
			MPU6050_DateConver(Accel);
			step_count1=step_count/48;
			sprintf ( cStr, "step_count  :%d",step_count1 );	
      ILI9341_DispStringLine_EN(LINE(16),cStr);
			 printf("步数:%d",step_count1);
			Delay_ms(500);
			
		}
	}
  
}

/*********************************************END OF FILE**********************/
