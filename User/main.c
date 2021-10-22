#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "./systick/bsp_SysTick.h"
#include "./led/bsp_led.h"
#include "./usart/bsp_usart.h"
#include "./mpu6050/mpu6050.h"
#include "./i2c/bsp_i2c.h"
#include "bsp_dht11.h"
#include "core_delay.h"


//�����Ƿ�ʹ��LCD������ʾ������Ҫ�Ļ��������ע�͵�����




#define USE_LCD_DISPLAY
#ifdef USE_LCD_DISPLAY
#include "./lcd/bsp_ili9341_lcd.h"
#endif

#define X_AXLE  0
#define Y_AXLE  1
#define Z_AXLE  2
#define DATA_COMPARE 21000 

/* MPU6050���� */
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
                //X���Ծ                 
                if((data_temp[i][X_AXLE]>=data_temp[i][Y_AXLE])&&(data_temp[i][X_AXLE]>=data_temp[i][Z_AXLE]))
                {
                    //����б�ʣ������ж�3��ֵ���ų����
                    if((data_temp[i][X_AXLE]-hold_value)<0) 
                        if((data_temp[i+1][X_AXLE]-hold_value)<0)
                            if((data_temp[i+2][X_AXLE]-hold_value)<0)
                        {
																	printf("data_tempY:%d\n",data_temp[i][X_AXLE]);
                                  if(data_temp[i][X_AXLE] > DATA_COMPARE)
                                    step_count++;
                        }
                }
                //Y���Ծ
                else  if((data_temp[i][Y_AXLE]>=data_temp[i][X_AXLE])&&(data_temp[i][Y_AXLE]>=data_temp[i][Z_AXLE]))
                {
                    //�����ж�3��ֵ���ų����
                    if((data_temp[i][X_AXLE]-hold_value)<0)
                        if((data_temp[i+1][X_AXLE]-hold_value)<0)
                            if((data_temp[i+2][X_AXLE]-hold_value)<0)
                        {             
																		printf("data_tempY:%d\n",data_temp[i][Y_AXLE]);
                                    if(data_temp[i][Y_AXLE] > DATA_COMPARE)
                                    step_count++;                        
                        }
                }
                //Z���Ծ
                else if((data_temp[i][Z_AXLE]>=data_temp[i][Y_AXLE])&&(data_temp[i][Z_AXLE]>=data_temp[i][X_AXLE]))
                {
                  //�����ж�3��ֵ���ų����
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
		
			//�������ֵ
			if ((++i_save)==50) 
			{
					//�ҳ����ֵ����Сֵ
					max=min=data_temp[0][0];
					for(i=0;i<50;i++)
							for(j=0;j<3;j++)
							{
									if(max<data_temp[i][j]) max=data_temp[i][j];
									if(min>data_temp[i][j]) min=data_temp[i][j];
							}
							//���㶯̬��ֵ
							hold_value = (max+min)/2;
							//���㲽��
						MPU6050_DataAnalyze(hold_value);
					//���²���
					i_save=0;
			}
	}
		
}




/**
  * @brief  ������
  * @param  ��  
  * @retval ��
  */
int main(void)
{
  char dispBuff[100];
  int k1;
	DHT11_Data_TypeDef DHT11_Data;
#ifdef USE_LCD_DISPLAY
  char cStr [ 70 ];
	ILI9341_Init ();         //LCD ��ʼ��
  ILI9341_GramScan ( 6 );
	LCD_SetFont(&Font8x16);
	LCD_SetColors(RED,BLACK);
	ILI9341_Clear ( 0, 0, 240, 320);	
  ILI9341_DispString_EN ( 20, 20, "This is a MPU6050 demo" );
#endif
  
  //��ʼ��systick
	SysTick_Init();
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;
	/* LED �˿ڳ�ʼ�� */
	LED_GPIO_Config();
	/* ����1ͨ�ų�ʼ�� */
	USART_Config();
  /*DHT11��ʼ��*/
	DHT11_Init ();
	//I2C��ʼ��
	i2c_GPIO_Config();
  //MPU6050��ʼ��
	MPU6050_Init();
  if (MPU6050ReadID() == 0)
  {
    printf("\r\nû�м�⵽MPU6050��������\r\n");
    LED_RED; 
    #ifdef USE_LCD_DISPLAY			
      /*����������ɫ������ı�����ɫ*/
      LCD_SetColors(BLUE,BLACK);	
      ILI9341_DispStringLine_EN(LINE(4),"No MPU6050 detected! ");			
      ILI9341_DispStringLine_EN(LINE(5),"Please check the hardware connection! ");		
    #endif
		while(1);	
	}
	


  
  /* ����SysTick��ʱ�����ж� */
  SysTick_Init(); //���� SysTick Ϊ 1ms �ж�һ�Σ����ж����ȡ����������
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //������ʱ��
  
  
  while(1)
  {
//    if( task_readdata_finish ) //task_readdata_finish = 1 ��ʾ��ȡMPU6050�������
//    {
//    
//      printf("\r\n���ٶȣ� %8d%8d%8d    ",Accel[0],Accel[1],Accel[2]);
//      printf("�����ǣ� %8d%8d%8d    ",Gyro[0],Gyro[1],Gyro[2]);
//      printf("�¶ȣ� %8.2f",Temp);
//			
//			
//				
//      #ifdef USE_LCD_DISPLAY	
//      ILI9341_DispStringLine_EN(LINE(7),"Acceleration");		
//      sprintf ( cStr, "%8d%8d%8d",Accel[0],Accel[1],Accel[2] );	//���ٶ�ԭʼ����
//      ILI9341_DispStringLine_EN(LINE(8),cStr);					

//      ILI9341_DispStringLine_EN(LINE(11),"Gyro        :");			
//      sprintf ( cStr, "%8d%8d%8d",Gyro[0],Gyro[1],Gyro[2] );	//��ԭʼ����
//      ILI9341_DispStringLine_EN(LINE(12),cStr);	
//			
//      sprintf ( cStr, "Temperture  :%8.2f",Temp );	//�¶�ֵ
//      ILI9341_DispStringLine_EN(LINE(15),cStr);

//      #endif
//      
//      task_readdata_finish = 0; // �����־λ
//    }
//		  Delay_ms(2000);
			/*����DHT11_Read_TempAndHumidity��ȡ��ʪ�ȣ����ɹ����������Ϣ*/
			if( DHT11_Read_TempAndHumidity ( & DHT11_Data ) == SUCCESS)
			{
        ILI9341_DispStringLine_EN(LINE(0),"YH DHT11 test");
        
        /* ��ʾ�¶� */
        sprintf(dispBuff,"Temperature : %d.%d ",DHT11_Data.temp_int, DHT11_Data.temp_deci);
        LCD_ClearLine(LINE(1));	/* ����������� */
        ILI9341_DispStringLine_EN(LINE(1),dispBuff);
        
        /* ��ʾʪ�� */
        sprintf(dispBuff,"Humidity : %d.%d%% ",DHT11_Data.humi_int, DHT11_Data.humi_deci);
        LCD_ClearLine(LINE(2));	/* ����������� */
        ILI9341_DispStringLine_EN(LINE(2),dispBuff);
			}			
			else
			{
        LCD_ClearLine(LINE(1));	/* ����������� */
        LCD_ClearLine(LINE(2));	/* ����������� */
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
			 printf("����:%d",step_count1);
			Delay_ms(500);
			
		}
	}
  
}

/*********************************************END OF FILE**********************/
