/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2015 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "zlg7290.h"
#include "i2c.h"
#include "RemoteInfrared.h"
#include <stdlib.h>
#include "Dc_motor.h"

IWDG_HandleTypeDef	hiwdg;
__IO uint32_t		GlobalTimingDelay100us;

#define ZLG_READ_ADDRESS1	0x01
#define ZLG_READ_ADDRESS2	0x10
#define ZLG_WRITE_ADDRESS1	0x10
#define ZLG_WRITE_ADDRESS2	0x11
#define BUFFER_SIZE1		(countof( Tx1_Buffer ) )
#define BUFFER_SIZE2		(countof( Rx2_Buffer ) )
#define countof( a ) (sizeof(a) / sizeof(*(a) ) )
#define I2C_Open_LONG_TIMEOUT         ((uint32_t)0xffff)

 __IO uint32_t  I2CTimeout1 = I2C_Open_LONG_TIMEOUT;

/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* uint32_t adcx[4]={0}; */
void switch_flag( int flag, int point );

void I2C_DC_Motor_WriteOneByte(I2C_HandleTypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t value)
{   
	while( HAL_I2C_Mem_Write(I2Cx, I2C_Addr, addr, I2C_MEMADD_SIZE_8BIT, &value, 0x01, I2CTimeout1) != HAL_OK ){};
		
}

void I2C_DC_Motor_Write(I2C_HandleTypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint8_t num)
{
	while(num--)
	{
		
    I2C_DC_Motor_WriteOneByte(I2Cx, I2C_Addr,addr++,*buf++);
		HAL_Delay(5);
		
	}
}

void Turn_On_LED( uint8_t LED_NUM );


__IO uint16_t	adcx[5][4] = { 0 };
uint8_t		Tx1_Buffer[8] = { 0 };
uint8_t		Rx2_Buffer[8] = { 0 };
uint8_t		state __attribute__( (section( "NO_INIT" ), zero_init) );
uint8_t		pre_state __attribute__( (section( "NO_INIT" ), zero_init) );
uint8_t		cnt __attribute__( (section( "NO_INIT" ), zero_init) );
/* variable */
float		t[6];
float		now_light = 0;
float       now_gas = 0;
float       now_alcohol = 0;
float       now_flame = 0;
float		last_light __attribute__( (section( "NO_INIT" ), zero_init) );
int		Red __attribute__( (section( "NO_INIT" ), zero_init) );
int		seq[4] __attribute__( (section( "NO_INIT" ), zero_init) );
uint8_t		buffer_check	= 0;
uint8_t		now_light_check = 0;
uint8_t		now_gas_check = 0;
uint8_t		now_alcohol_check = 0;
uint8_t		now_flame_check = 0;
uint32_t	unStartFlag __attribute__( (section( "NO_INIT" ), zero_init) );
int sensor_type __attribute__( (section( "NO_INIT" ), zero_init) );
int sensor_type_backup[3] __attribute__( (section( "NO_INIT" ), zero_init) );

int enBeep = 0;

/* USER CODE END 4 */
uint32_t tick_count = 0;

/* backup data struct */
typedef struct backup {
    uint8_t Rx2_Buffer[8];
    float	now_light;
    float   now_gas;
    float   now_alcohol;
    float   now_flame;
    uint8_t checksum;
} Backup;

Backup backup_data[3] __attribute__( (section( "NO_INIT" ), zero_init) );
/* USER CODE END PV */

void SystemClock_Config( void );


void MX_IWDG_Init( void )
{
    hiwdg.Instance		= IWDG;
    hiwdg.Init.Prescaler	= IWDG_PRESCALER_128;
    hiwdg.Init.Reload	= 0x3FF;
    HAL_IWDG_Init( &hiwdg );
}


void HAL_IWDG_MspInit( IWDG_HandleTypeDef* hiwdg )
{
}


void MX_IWDG_Start( void )
{
    HAL_IWDG_Start( &hiwdg );
}


void MX_IWDG_Refresh( void )
{
    HAL_IWDG_Refresh( &hiwdg );
}


/* backup function definitions */
void update_checksum_num(int num)
{
    uint8_t * ckpt	= (uint8_t *) &backup_data[num];
    int	len	= sizeof(backup_data[num]);
    uint8_t temp	= 0;

    for ( int i = 0; i < len - 1; i++ )
    {
        temp ^= ckpt[i];
    }
    backup_data[num].checksum = temp;
}


void set_buffer( uint8_t Buffer_value[8] )
{
    for ( int i = 0; i < 8; i++ ) /* 更新Rx2_Buffer */
    {
        Rx2_Buffer[i] = Buffer_value[i];
    }

    /* 更新buffer_check */
    uint8_t temp = 0;
    for ( int i = 0; i < 8; i++ )
    {
        temp ^= Buffer_value[i];
    }
    buffer_check = temp;

    for ( int i = 0; i < 8; i++ )   /* 更新备份数据 */
    {
        backup_data->Rx2_Buffer[i] = Buffer_value[i];
        update_checksum_num(i);              /* 更新backup_data->checksum */
    }
    
}


void set_now_light( float now_light_value )
{
    now_light = now_light_value;    /* 更新now_light */

    /* 更新now_light_check */
    uint8_t * pt	= (uint8_t *) &now_light_value;
    uint8_t temp	= 0;
    for ( int i = 0; i < sizeof(now_light_value); i++ )
    {
        temp ^= pt[i];
    }
    now_light_check = temp;

    for (int i = 0; i < 3 ; i++){
        backup_data[i].now_light = now_light_value;       /* 更新备份数据 */  
        update_checksum_num(i);                              /* 更新backup_data->checksum */    
    }
}

void set_now_gas( float now_gas_value )
{
    now_gas = now_gas_value;    /* 更新now_light */

    uint8_t * pt	= (uint8_t *) &now_gas_value;
    uint8_t temp	= 0;
    for ( int i = 0; i < sizeof(now_gas_value); i++ )
    {
        temp ^= pt[i];
    }
    now_gas_check = temp;

    for (int i = 0; i < 3 ; i++){
        backup_data[i].now_gas = now_gas_value;       /* 更新备份数据 */
        update_checksum_num(i);                              /* 更新backup_data->checksum */
    }
}

void set_now_alcohol( float now_alcohol_value )
{
    now_alcohol = now_alcohol_value;    /* 更新now_light */

    /* 更新now_light_check */
    uint8_t * pt	= (uint8_t *) &now_alcohol_value;
    uint8_t temp	= 0;
    for ( int i = 0; i < sizeof(now_alcohol_value); i++ )
    {
        temp ^= pt[i];
    }
    now_alcohol_check = temp;

    for (int i = 0; i < 3 ; i++){
        backup_data[i].now_alcohol = now_alcohol_value;       /* 更新备份数据 */
        update_checksum_num(i);                              /* 更新backup_data->checksum */
    }
}

void set_now_flame( float now_flame_value )
{
    now_flame = now_flame_value;    /* 更新now_light */

    /* 更新now_light_check */
    uint8_t * pt	= (uint8_t *) &now_flame_value;
    uint8_t temp	= 0;
    for ( int i = 0; i < sizeof(now_flame_value); i++ )
    {
        temp ^= pt[i];
    }
    now_flame_check = temp;

    for (int i = 0; i < 3 ; i++){
        backup_data[i].now_flame = now_flame_value;       /* 更新备份数据 */
        update_checksum_num(i);                              /* 更新backup_data->checksum */
    }
}

int verify_checksum_num(int num)   /* 校验整个备份数据结构体 */
{
    uint8_t * ckpt	= (uint8_t *) &backup_data[num];
    int	len	= sizeof(backup_data);
    uint8_t temp	= 0;

    for ( int i = 0; i < len; i++ )
    {
        temp ^= ckpt[i];
    }

    return(temp); /* 返回0说明校验通过 */
}

int verify_checksum()
{
    int valid[3] = {0};
    int valid_index = -1;
    for(int i = 0; i < 3; i++){
        if(verify_checksum_num(i) == 0){
            valid[i] = 1;
            valid_index = i;
        }
    }
    if(valid_index == -1){
        return 0; /* 冷启动 */
    }
    for(int i = 0; i < 3; i++){
        if(valid[i] == 0){
            for (int j = 0; j < 8; j++){
                backup_data[i].Rx2_Buffer[j] = backup_data[valid_index].Rx2_Buffer[j];
            }
            backup_data[i].now_light = backup_data[valid_index].now_light;
            backup_data[i].now_gas = backup_data[valid_index].now_gas;
            backup_data[i].now_alcohol = backup_data[valid_index].now_alcohol;
            backup_data[i].now_flame = backup_data[valid_index].now_flame;
            return 0;
        }
    }
    set_buffer( backup_data[valid_index].Rx2_Buffer );
    set_now_light( backup_data[valid_index].now_light );
    set_now_gas(backup_data[valid_index].now_gas);
    set_now_alcohol(backup_data[valid_index].now_alcohol);
    set_now_flame(backup_data[valid_index].now_flame);
    return 1;
}


uint8_t* get_buffer()
{
    uint8_t temp1 = 0;
    for ( int i = 0; i < 8; i++ )
    {
        temp1 ^= Rx2_Buffer[i];
    }

    if ( temp1 == buffer_check ) /* 校验当前数据 */
    { /* set_buffer(Rx2_Buffer); */
    for (int i = 0; i < 3 ; i++){
            for ( int j = 0; j < 8; j++ )
            {
                backup_data[i].Rx2_Buffer[j] = Rx2_Buffer[j];
            }
        }
        return(Rx2_Buffer);
    }else {
        if (verify_checksum() == 0)
        {
            HAL_NVIC_SystemReset();         /* 校验不通过，冷启动 */
        }
    }
}


float get_now_light()
{
    uint8_t * pt	= (uint8_t *) &now_light;
    uint8_t temp1	= 0;
    for ( int i = 0; i < sizeof(now_light); i++ )
    {
        temp1 ^= pt[i];
    }

    if ( temp1 == now_light_check ) /* 校验当前数据 */
    {
        for (int i = 0; i < 3 ; i++){
            /* set_now_light(now_light); */
            backup_data[i].now_light = now_light;
        }
        return(now_light);
    }else    {
        if (verify_checksum() == 0)
        {
            HAL_NVIC_SystemReset();         /* 校验不通过，冷启动 */
        }            
    }
}

float get_now_gas()
{
    uint8_t * pt	= (uint8_t *) &now_gas;
    uint8_t temp1	= 0;
    for ( int i = 0; i < sizeof(now_gas); i++ )
    {
        temp1 ^= pt[i];
    }

    if ( temp1 == now_gas_check ) /* 校验当前数据 */
    {
        /* set_now_light(now_light); */
        for (int i = 0; i < 3 ; i++){
            backup_data[i].now_gas = now_gas;
            update_checksum_num(i);
        }

        return(now_gas);
    }else    {
        if (verify_checksum() == 0)
        {
            HAL_NVIC_SystemReset();         /* 校验不通过，冷启动 */
        }            
    }
}

float get_now_alcohol()
{
    uint8_t * pt	= (uint8_t *) &now_alcohol;
    uint8_t temp1	= 0;
    for ( int i = 0; i < sizeof(now_alcohol); i++ )
    {
        temp1 ^= pt[i];
    }

    if ( temp1 == now_alcohol_check ) /* 校验当前数据 */
    {
        for (int i = 0; i < 3 ; i++){
            backup_data[i].now_alcohol = now_alcohol;
            update_checksum_num(i);
        }

        return(now_alcohol);
    }else  {
        if (verify_checksum() == 0)
        {
            HAL_NVIC_SystemReset();         /* 校验不通过，冷启动 */
        }
    }
}

float get_now_flame()
{
    uint8_t * pt	= (uint8_t *) &now_flame;
    uint8_t temp1	= 0;
    for ( int i = 0; i < sizeof(now_flame); i++ )
    {
        temp1 ^= pt[i];
    }

    if ( temp1 == now_flame_check ) /* 校验当前数据 */
    {
        for (int i = 0; i < 3 ; i++){
            backup_data[i].now_flame = now_flame;
            update_checksum_num(i);
        }

        return(now_flame);
    }else    {
        if (verify_checksum() == 0)
        {
            HAL_NVIC_SystemReset();         /* 校验不通过，冷启动 */
        }
    }
}


void read_light()
{
    float	maxt	= 0, mint = 10000000;
    float	sum	= 0;
    for ( int i = 0; i < 5; i++ )
    {
        t[i] = (float) adcx[i][1];
        if ( maxt < t[i] )
            maxt = t[i];
        if ( mint > t[i] )
            mint = t[i];
        sum += t[i];
    }
    set_now_light( (sum - maxt - mint) / 3 * (3.3 / 4096) );
}

void read_gas()
{
    float	maxt	= 0, mint = 10000000;
    float	sum	= 0;
    for ( int i = 0; i < 5; i++ )
    {
        t[i] = (float) adcx[i][3];
        if ( maxt < t[i] )
            maxt = t[i];
        if ( mint > t[i] )
            mint = t[i];
        sum += t[i];
    }
    set_now_gas( (sum - maxt - mint) / 3 * (3.3 / 4096) );
}

void read_alcohol()
{
    float	maxt	= 0, mint = 10000000;
    float	sum	= 0;
    for ( int i = 0; i < 5; i++ )
    {
        t[i] = (float) adcx[i][0];
        if ( maxt < t[i] )
            maxt = t[i];
        if ( mint > t[i] )
            mint = t[i];
        sum += t[i];
    }
    set_now_alcohol( (sum - maxt - mint) / 3 * (3.3 / 4096) );
}

void read_flame()
{
    float	maxt	= 0, mint = 10000000;
    float	sum	= 0;
    for ( int i = 0; i < 5; i++ )
    {
        t[i] = (float) adcx[i][2];
        if ( maxt < t[i] )
            maxt = t[i];
        if ( mint > t[i] )
            mint = t[i];
        sum += t[i];
    }
    set_now_flame( (sum - maxt - mint) / 3 * (3.3 / 4096) );
}

void read_all_sensors()
{
    read_alcohol();
    read_light();
    read_gas();
    read_flame();
}


void show_sensor_data()
{
    
    // 根据不同传感器类型，调整显示格式（倍率和小数点位置）
    int scaling_factor = 1000000;  // 默认光敏传感器的倍率
    int decimal_pos = 6;            // 小数点位置
    
	printf("sensor:%d\n",sensor_type);
    switch(sensor_type)
    {
        case 0: // 酒精
            scaling_factor = 1000000;
            decimal_pos = 6;
            break;
        case 1: // 光敏
            scaling_factor = 1000000;
            decimal_pos = 6;
            break;
        case 2: // 火焰
            scaling_factor = 1000000;
            decimal_pos = 6;
            break;
        case 3: // 气体
            scaling_factor = 1000000;
            decimal_pos = 6;
            break;
    }
    
    int tmp = 0;
    // 在最前面显示传感器类型
    Tx1_Buffer[0] = 0x00;
    I2C_ZLG7290_Write( &hi2c1, 0x70, ZLG_WRITE_ADDRESS1, Tx1_Buffer, 8 );
    
    // 第一位显示传感器类型：A-酒精，L-光敏，F-火焰，G-气体
    switch(sensor_type)
    {
        case 0: // 酒精 - 显示字母A
            tmp = now_alcohol * scaling_factor;
            Tx1_Buffer[1] = 0xEE; // 字母A的段码
            break;
        case 1: // 光敏 - 显示字母L
            tmp = now_light * scaling_factor;
            Tx1_Buffer[1] = 0x70; // 字母L的段码
            break;
        case 2: // 火焰 - 显示字母F
            tmp = now_flame * scaling_factor;
            Tx1_Buffer[1] = 0x8E; // 字母F的段码
            break;
        case 3: // 气体 - 显示字母G
            tmp = now_gas * scaling_factor;
            Tx1_Buffer[1] = 0xBC; // 字母G的段码
            break;
        default:
				{
            int tmp_backup[3] = {0};
            for(int i = 0;i<3;i++)
            {
                tmp_backup[sensor_type_backup[i]]++;
            }
            for(int i = 0;i<3;i++)
            {
                if(tmp_backup[i] >= 2)
                    sensor_type = tmp_backup[i];
            }
            if(sensor_type != 0 && sensor_type != 1 && sensor_type != 2 && sensor_type != 3)
                HAL_NVIC_SystemReset();
            break;
					}
    }
    

		
    
    // 显示数值
    for (int i = 0; i < 7; i++)
    {
        int flag = tmp % 10;
        tmp /= 10;
        
        if (i == decimal_pos)
            switch_flag(flag, 1);  // 加小数点
        else
            switch_flag(flag, 0);
            
        Rx2_Buffer[7-i] = Tx1_Buffer[0];
        set_buffer(Rx2_Buffer);
    }
    
		Rx2_Buffer[0] = Tx1_Buffer[1];
    set_buffer(Rx2_Buffer);
		
    I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS1, Rx2_Buffer, BUFFER_SIZE2);
		
}

void Turn_On_LED( uint8_t LED_NUM )
{
    switch ( LED_NUM )
    {
    case 3:
        HAL_GPIO_WritePin( GPIOH, GPIO_PIN_15, GPIO_PIN_RESET );        /*点亮D4灯*/
        break;
    case 2:
        HAL_GPIO_WritePin( GPIOB, GPIO_PIN_15, GPIO_PIN_RESET );        /*点亮D3灯*/
        break;
    case 1:
        HAL_GPIO_WritePin( GPIOC, GPIO_PIN_0, GPIO_PIN_RESET );         /*点亮D2灯*/
        break;
    case 0:
        HAL_GPIO_WritePin( GPIOF, GPIO_PIN_10, GPIO_PIN_RESET );        /*点亮D1灯*/
        break;
    default:
        break;
    }
}

void Turn_Off_LED( uint8_t LED_NUM )
{
    switch ( LED_NUM )
    {
    case 3:
        HAL_GPIO_WritePin( GPIOH, GPIO_PIN_15, GPIO_PIN_SET );        /*关闭D4灯*/
        break;
    case 2:
        HAL_GPIO_WritePin( GPIOB, GPIO_PIN_15, GPIO_PIN_SET );        /*关闭D3灯*/
        break;
    case 1:
        HAL_GPIO_WritePin( GPIOC, GPIO_PIN_0, GPIO_PIN_SET );         /*关闭D2灯*/
        break;
    case 0:
        HAL_GPIO_WritePin( GPIOF, GPIO_PIN_10, GPIO_PIN_SET );        /*关闭D1灯*/
        break;
    default:
        break;
    }
}


void marquee()
{
	/* 跑马灯 */

//酒精
	
			if ( 2 * get_now_alcohol() > 3 ){
				Turn_On_LED( 0 );
				enBeep = enBeep | 1;
			}
			else
			{
				Turn_Off_LED( 0 );
				enBeep = enBeep & ~1;
			}
 //光敏
			if ( get_now_light() > 2 )
			{
				Turn_On_LED( 1 );
				enBeep = enBeep | (1 << 1);
			}
			else
			{
				Turn_Off_LED( 1 );
				enBeep = enBeep & ~(1 << 1);
			}
 //火焰
			if ( 2 * get_now_flame() > 5 )
			{
				Turn_On_LED( 2 );
				enBeep = enBeep | (1 << 2);
			}
			else{
				Turn_Off_LED( 2 );
				enBeep = enBeep & ~(1 << 2);
			}
 //气体
			if ( get_now_gas() < 0.035 )
			{
				Turn_On_LED( 3 );
				enBeep = enBeep | (1 << 3);
			}
			else
			{
				Turn_Off_LED( 3 );
				enBeep = enBeep & ~(1 << 3);
			}


}


uint8_t fs_flag = 0;
void DC_Task(uint8_t iKey)
{
	uint8_t Buffer_DC[1]={0Xff};
	uint8_t Buffer_DC_Zero[1]={0x00};
	switch(iKey)
	{
        case 0x1C:						 //1
					DC_Motor_Pin_Low();	
					fs_flag = 1;  //zheng
					I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x0F,Buffer_DC_Zero,1);				
          break;
        case 0x1B:							//2
					DC_Motor_Pin_Low();	
					fs_flag = 1;  //zheng
				  I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x03,Buffer_DC,1);
          break;
        case 0x1A:							//3
					DC_Motor_Pin_Low();	
					fs_flag = 1;  //zheng
					I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x0f,Buffer_DC,1);
          break;			
        case 0x14:					//4
				  DC_MOtor_Pin_High();
				  fs_flag = 0;  //zheng
					I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x0A,Buffer_DC,1);				
          break;   
				case 0x13:							//5		
				  DC_MOtor_Pin_High();
				  fs_flag = 0;  //zheng
				  I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x05,Buffer_DC,1);
					break;
        case 0x12:							//6
					DC_MOtor_Pin_High();
				  fs_flag = 0;  //zheng 
				  I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x03,Buffer_DC,1);
          break;
				
        default:
					DC_Motor_Pin_Low();	
				   fs_flag = 2;  //zheng
				  I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x00,Buffer_DC_Zero,1);
          break;
			}
}

void motor_v()
{
    /* 直流电机 */
    if ( state )
    {
        if ( 2 * get_now_light() > 3 && get_now_light() <= 2)
            DC_Task(0x12);
        if ( get_now_light() > 2 && 2 * get_now_light() <= 5)
            DC_Task(0x13);
        if ( 2 * get_now_light() > 5)
            DC_Task(0x14);
				else DC_Task(0x00);

    }else  {
        DC_Task(0x00);
    }
}

void update_state_and_cnt()
{
    if ( pre_state == HAL_GPIO_ReadPin( GPIOG, GPIO_PIN_9 ) )       /* 更新state和cnt */
    {
        cnt += 1;
        if ( cnt > 20 )                                         /* 防止cnt溢出 */
            cnt = 20;
    }else  {
        cnt		= 1;
        pre_state	= HAL_GPIO_ReadPin( GPIOG, GPIO_PIN_9 );
    }
    if ( cnt >= 3 )
        state = pre_state;
    printf( "State %d, cnt %d;\n\n", state, cnt );
    printf("[传感器数据]\n");
    printf("酒精传感器: %1.7f\n", now_alcohol);
    printf("光敏传感器: %1.7f\n", now_light);
    printf("气体传感器: %1.7f\n", now_gas);
		printf("火焰传感器: %1.7f\n", now_flame);
		printf("startflag=%x\n",unStartFlag);
		
		printf("%d, %d\n", enBeep, tick_count);
		printf("%d\n",HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_6));
		
}

uint8_t flag;//不同的按键有不同的标志位值
uint8_t flag1 = 0;//中断标志位，每次按键产生一次中断，并开始读取8个数码管的值
void swtich_key(void);
uint8_t Rx1_Buffer[1]={0};
int main( void )
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    int initsum = rand() % 3;
    /* Configure the system clock */
    SystemClock_Config();
    if ( unStartFlag == 0xAA55AA55 && verify_checksum() == 1 )
    {
    }else{ /* 冷启动 */
			   if(initsum == 0){
						 cnt		= 0;
						 pre_state	= 0;
						 state		= 0;
						 HAL_Delay( 20000 );
						 //backup_data	= (Backup *) malloc( sizeof(Backup) );
						 unStartFlag	= 0xAA55AA55;
					 sensor_type = 0;
				 }
				 else if(initsum == 1){
						 cnt		= 0;
						 pre_state	= 0;
						 state		= 0;
						 HAL_Delay( 20000 );
						 unStartFlag	= 0xAA55AA55;
					   //backup_data	= (Backup *) malloc( sizeof(Backup) );
					 sensor_type = 0;
				 }
				 else if(initsum == 2){
						 cnt		= 0;
						 pre_state	= 0;
						 state		= 0;
						 unStartFlag	= 0xAA55AA55;
					   HAL_Delay( 20000 );
					 sensor_type = 0;
						 //backup_data	= (Backup *) malloc( sizeof(Backup) );
				 }
    }

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC3_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
	  
		// 

    MX_IWDG_Init();                                         /* 看门狗程序初始化 */

    for(int i = 0;i < 5;i ++) HAL_ADC_Start_DMA( &hadc3, (uint32_t *) adcx[i], 4 );      /* 开启ADC转换 */
		
		// HAL_Delay( 100000 );
    MX_IWDG_Start();                                        /* 开启看门狗 */
    set_buffer( Rx2_Buffer );
    
		printf("\n\r");
    printf("\n\r-------------------------------------------------\r\n");
    printf("\n\r 多传感器显示与按键切换系统\r\n");
    printf("\n\r 按键1-4分别切换不同传感器的显示\r\n");
    printf("\n\r-------------------------------------------------\r\n");
    
		
		
		
		
		int initcnt = 0;
    /* Infinite loop */
    while ( 1 )
    {
			    /* USER CODE BEGIN 3 */
				if(flag1 == 1)
				{
					flag1 = 0;
					I2C_ZLG7290_Read(&hi2c1,0x71,0x01,Rx1_Buffer,1);//读键值
					printf("\n\r按键键值 = %#x\r\n",Rx1_Buffer[0]);//想串口发送键值
					swtich_key();//扫描键值，写标志位
				}		

        //Remote_Infrared_KeyDeCode();
			  Red = 1;
        MX_IWDG_Refresh();
			  
				for(int initcnt = 0;initcnt<10;initcnt++)
				{
			  if(initcnt % 2 == 0){

					MX_GPIO_Init();  
					MX_DMA_Init();
					MX_ADC3_Init();
					MX_I2C1_Init();
					MX_USART1_UART_Init();
					HAL_ADC_Start_DMA( &hadc3, (uint32_t *) adcx[(initcnt)/2], 4);
				}					
				}

        for ( int i = 0; i < 4; i++ )
            seq[i] = 0;
			  motor_v();
        int r = rand() % 2;
        if ( r == 0 )
        {
            read_all_sensors();
            seq[0] = 1;
            if ( seq[0] == 1 )
            {
                if ( Red )
                    show_sensor_data();
                else{
                    Tx1_Buffer[0] = 0x00; 
                    I2C_ZLG7290_Write( &hi2c1, 0x70, ZLG_WRITE_ADDRESS1, Tx1_Buffer, 8 );
                }
                seq[1] = 2;
            }
            if ( seq[0] == 1 && seq[1] == 2 )
            {
                marquee();
                seq[2] = 3;
            }
            if ( seq[0] == 1 && seq[1] == 2 && seq[2] == 3 )
            {
                update_state_and_cnt();
            }
                
        }else  {
            read_all_sensors();
            seq[0] = 1;
            if ( seq[0] == 1 )
            {
                update_state_and_cnt();
                seq[1] = 4;
            }
            if ( seq[0] == 1 && seq[1] == 4 )
            {
                marquee();
                seq[2] = 3;
            }
            if ( seq[0] == 1 && seq[1] == 4 && seq[2] == 3 ){
                if ( Red )
                    show_sensor_data();
                else{
                    Tx1_Buffer[0] = 0x00; 
                    I2C_ZLG7290_Write( &hi2c1, 0x70, ZLG_WRITE_ADDRESS1, Tx1_Buffer, 8 );
                }
							}

        }
     HAL_Delay( 10000 );

    }
}
/* USER CODE BEGIN 4 */
void swtich_key(void)
{
	switch(Rx1_Buffer[0])
	{
        case 0x1C:
					flag = 1;
					sensor_type = 0;
                    for(int i = 0;i<3;i++)
                    {
                        sensor_type_backup[i] = 0;
                    }		
          break;
        case 0x1B:	
					flag = 2;
				sensor_type = 1;	
                for(int i = 0;i<3;i++)
                {
                    sensor_type_backup[i] = 1;
                }		
          break;
        case 0x1A:	
					flag = 3;
				sensor_type = 2;
                for(int i = 0;i<3;i++)
                {
                    sensor_type_backup[i] = 2;
                }			
          break;
        case 0x14:
					flag = 4;
				sensor_type = 3;
                for(int i = 0;i<3;i++)
                {
                    sensor_type_backup[i] = 3;
                }			
          break;   
				case 0x13:
					flag = 5;
					break;
        case 0x12:
					flag = 6;
          break;
        case 0x0C:
					flag = 7;
          break;
        case 0x0B:
          flag = 8;
          break;
				case 0x0A:
					flag = 9;
					break;
				case 0x03:
					flag = 15;
					break;
				case 0x19:
					flag = 10;
					break;
				case 0x11:
					flag = 11;
					break;
				case 0x09:
					flag = 12;
					break;
				case 0x01:
					flag = 13;
					break;
				case 0x02:
					flag = 14;
					break;
        default:	
          break;
			}
}

    void switch_flag( int flag, int point )
    {
        switch ( flag )
        {
        case 0:
            Tx1_Buffer[0] = 0xFC;
            break;
        case 1:
            Tx1_Buffer[0] = 0x0c;
            break;
        case 2:
            Tx1_Buffer[0] = 0xDA;
            break;
        case 3:
            Tx1_Buffer[0] = 0xF2;
            break;
        case 4:
            Tx1_Buffer[0] = 0x66;
            break;
        case 5:
            Tx1_Buffer[0] = 0xB6;
            break;
        case 6:
            Tx1_Buffer[0] = 0xBE;
            break;
        case 7:
            Tx1_Buffer[0] = 0xE0;
            break;
        case 8:
            Tx1_Buffer[0] = 0xFE;
            break;
        case 9:
            Tx1_Buffer[0] = 0xE6;
            break;
        case 10:
            Tx1_Buffer[0] = 0xEE;
            break;
        case 11:
            Tx1_Buffer[0] = 0x3E;
            break;
        case 12:
            Tx1_Buffer[0] = 0x9C;
            break;
        case 13:
            Tx1_Buffer[0] = 0x7A;
            break;
        case 14: /* #?z? */
            Tx1_Buffer[0] = 0x00;
            I2C_ZLG7290_Write( &hi2c1, 0x70, ZLG_WRITE_ADDRESS1, Tx1_Buffer, 8 );
            break;
        case 15:
            Tx1_Buffer[0] = 0xFC;
            break;
        default:
            break;
        }
        if ( point ) /* ?????????? */
        {
            Tx1_Buffer[0] |= 0x1;
        }
    }


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	flag1 = 1;
	printf("flag = %d",flag1);
}

/** System Clock Configuration **/
void SystemClock_Config( void )  /* modified */
{
    RCC_OscInitTypeDef	RCC_OscInitStruct;
    RCC_ClkInitTypeDef	RCC_ClkInitStruct;

    __PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    RCC_OscInitStruct.OscillatorType	= RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState		= RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue	= 16;
    RCC_OscInitStruct.HSEState		= RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState		= RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource		= RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM		= 25;
    RCC_OscInitStruct.PLL.PLLN		= 336;
    RCC_OscInitStruct.PLL.PLLP		= RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ		= 4;
    HAL_RCC_OscConfig( &RCC_OscInitStruct );

    RCC_ClkInitStruct.ClockType		= RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource		= RCC_SYSCLKSOURCE_HSI | RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider		= RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider	= RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider	= RCC_HCLK_DIV2;
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_5 );

    HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq() / 10000 );

    HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}


/* USER CODE BEGIN 4 */
int fputc( int ch, FILE *f )
{
    while ( (USART1->SR & 0X40) == 0 )
        ;            /* 循环发送,直到发送完毕 */
    USART1->DR = (uint8_t) ch;
    return(ch);
}


int onoff;

void HAL_SYSTICK_Callback( void )
{
    if ( GlobalTimingDelay100us != 0 )
    {
        GlobalTimingDelay100us--;
    }
		if (enBeep && tick_count >= 10)
		//if (enBeep)
		{
			tick_count = 0;
			if(onoff == 0)
			{
				//HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,~HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_6));//´ò¿ª·äÃùÆ÷
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
				onoff = 1;
			}
			else
			{
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);
				onoff = 0;
			}
			
		}
		tick_count ++ ;
}


#ifdef USE_FULL_ASSERT


/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */

void assert_failed( uint8_t* file, uint32_t line )
{
    /* USER CODE BEGIN 6 */


    /* User can add his own implementation to report the file name and line number,
     * ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}


#endif


/**
 * @}
 */


/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
