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

IWDG_HandleTypeDef	hiwdg;
__IO uint32_t		GlobalTimingDelay100us;

#define ZLG_READ_ADDRESS1	0x01
#define ZLG_READ_ADDRESS2	0x10
#define ZLG_WRITE_ADDRESS1	0x10
#define ZLG_WRITE_ADDRESS2	0x11
#define BUFFER_SIZE1		(countof( Tx1_Buffer ) )
#define BUFFER_SIZE2		(countof( Rx2_Buffer ) )
#define countof( a ) (sizeof(a) / sizeof(*(a) ) )

/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* uint32_t adcx[4]={0}; */
void switch_flag( int flag, int point );


void Turn_On_LED( uint8_t LED_NUM );


__IO uint16_t	adcx[4] = { 0 };
uint8_t		Tx1_Buffer[8] = { 0 };
uint8_t		Rx2_Buffer[8] = { 0 };
uint8_t		state __attribute__( (section( "NO_INIT" ), zero_init) );
uint8_t		pre_state __attribute__( (section( "NO_INIT" ), zero_init) );
uint8_t		cnt __attribute__( (section( "NO_INIT" ), zero_init) );
/* variable */
float		t[6];
float		now_light = 0;
float		last_light __attribute__( (section( "NO_INIT" ), zero_init) );
int		Red __attribute__( (section( "NO_INIT" ), zero_init) );
int		seq[4] __attribute__( (section( "NO_INIT" ), zero_init) );
uint8_t		buffer_check	= 0;
uint8_t		now_light_check = 0;
uint32_t	unStartFlag __attribute__( (section( "NO_INIT" ), zero_init) );

/* backup data struct */
typedef struct backup {
    uint8_t Rx2_Buffer[8];
    float	now_light;
    uint8_t checksum;
} Backup;

Backup* backup_data;
/* USER CODE END PV */

void SystemClock_Config( void );


void MX_IWDG_Init( void )
{
    hiwdg.Instance		= IWDG;
    hiwdg.Init.Prescaler	= IWDG_PRESCALER_128;
    hiwdg.Init.Reload	= 0x7FFFFFFF;
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
void update_checksum()
{
    uint8_t * ckpt	= (uint8_t *) backup_data;
    int	len	= sizeof(backup_data);
    uint8_t temp	= 0;

    for ( int i = 0; i < len - 1; i++ )
    {
        temp ^= ckpt[i];
    }
    backup_data->checksum = temp;
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
    }
    update_checksum();              /* 更新backup_data->checksum */
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

    backup_data->now_light = now_light_value;       /* 更新备份数据 */
    update_checksum();                              /* 更新backup_data->checksum */
}


void recovery_handle()
{
    set_buffer( backup_data->Rx2_Buffer );
    set_now_light( backup_data->now_light );
    printf( "[Warning] data is broken, recovering from heap\n" );
}


int verify_checksum()   /* 校验整个备份数据结构体 */
{
    uint8_t * ckpt	= (uint8_t *) backup_data;
    int	len	= sizeof(backup_data);
    uint8_t temp	= 0;

    for ( int i = 0; i < len; i++ )
    {
        temp ^= ckpt[i];
    }

    return(temp); /* 返回0说明校验通过 */
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
        for ( int j = 0; j < 8; j++ )
        {
            backup_data->Rx2_Buffer[j] = Rx2_Buffer[j];
        }
        update_checksum();
        return(Rx2_Buffer);
    }else    {
        int temp2 = verify_checksum();          /* 当前数据校验不通过，校验备份数据 */
        if ( temp2 == 0 )
        {
            recovery_handle();              /* 备份数据校验通过，使用备份数据更新Rx2_Buffer */
            printf( "[Warning] stack data 'Rx2_Buffer' is broken, recovering from heap\n" );
            return(Rx2_Buffer);
        }else
            HAL_NVIC_SystemReset();         /* 校验不通过，冷启动 */
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
        /* set_now_light(now_light); */
        backup_data->now_light = now_light;
        update_checksum();

        return(now_light);
    }else    {
        int temp2 = verify_checksum();          /* 当前数据校验不通过，校验备份数据 */
        if ( temp2 == 0 )
        {
            recovery_handle();              /* 备份数据校验通过，使用备份数据更新now_light */
            printf( "[Warning] stack data 'now_light' is broken, recovering from heap\n" );
            return(now_light);
        }else
            HAL_NVIC_SystemReset();         /* 校验不通过，冷启动 */
    }
}


void read_light()
{
    float	maxt	= 0, mint = 10000000;
    float	sum	= 0;
    for ( int i = 0; i < 5; i++ )
    {
        t[i] = (float) adcx[1];
        if ( maxt < t[i] )
            maxt = t[i];
        if ( mint > t[i] )
            mint = t[i];
        sum += t[i];
    }
    set_now_light( (sum - maxt - mint) / 3 * (3.3 / 4096) );
}


void show_LED()
{
    /* 光敏数值显示到晶体管 */
    int tmp = now_light * 10000000;
    Tx1_Buffer[0] = 0x00;                           /* 清除晶体管 */
    I2C_ZLG7290_Write( &hi2c1, 0x70, ZLG_WRITE_ADDRESS1, Tx1_Buffer, 8 );
    for ( int i = 0; i < 8; i++ )
    {
        int flag = tmp % 10;
        tmp /= 10;
        if ( i == 7 )
            switch_flag( flag, 1 );         /* 扫描到相应的按键并且向数码管写进数值,加小数点 */
        else
            switch_flag( flag, 0 );
        Rx2_Buffer[7 - i] = Tx1_Buffer[0];
        set_buffer( Rx2_Buffer );
    }
    I2C_ZLG7290_Write( &hi2c1, 0x70, ZLG_WRITE_ADDRESS2, Rx2_Buffer, BUFFER_SIZE2 );
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
    if ( state )
    {
        if ( 2 * get_now_light() > 3 )
            Turn_On_LED( 0 );
        else
            Turn_Off_LED( 0 );
        if ( get_now_light() > 2 )
            Turn_On_LED( 1 );
        else
            Turn_Off_LED( 1 );
        if ( 2 * get_now_light() > 5 )
            Turn_On_LED( 2 );
        else
            Turn_Off_LED( 2 );
        if ( get_now_light() > 3 )
            Turn_On_LED( 3 );
        else
            Turn_Off_LED( 3 );
    }else  {
        Turn_Off_LED( 1 );
        Turn_Off_LED( 2 );
        Turn_Off_LED( 3 );
        Turn_Off_LED( 0 );
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
    if ( get_now_light() != last_light )
    {
        printf( "[SysInfo]:Light is changed, now light is : %lf\n", get_now_light() );
    }
    last_light = get_now_light();
}


int main( void )
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    if ( unStartFlag == 0xAA55AA55 && verify_checksum() == 0 )
    {
    }else  { /* 冷启动 */
        Red		= 0;
        cnt		= 0;
        pre_state	= 0;
        state		= 0;
        HAL_Delay( 10000 );
        backup_data	= (Backup *) malloc( sizeof(Backup) );
        unStartFlag	= 0xAA55AA55;
    }

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC3_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();


    MX_IWDG_Init();                                         /* 看门狗程序初始化 */

    HAL_ADC_Start_DMA( &hadc3, (uint32_t *) adcx, 4 );      /* 开启ADC转换 */
    printf( "本实验实现一个安全可控的光敏装置，在基础功能方面，我们通过光敏传感器监测当前的光照强度，光敏传感器的数值越大说明" );
    printf( "光强越小，我们利用光电开关模拟检测人的经过，当光电开关处被阻挡，说明有人经过。" );
    printf( "此时根据光敏传感器的数值打开跑马灯，具体来说，我们将光敏传感器数值分为4档。(1.5, 2], (2, 2.5], (2.5, 3], (3, 3.5]，光强从1~4档逐步降低。" );
    printf( "当光电开关被阻挡，根据档次开启对应数量的跑马灯。1~4档分别对应1~4盏灯。" );

    MX_IWDG_Start();                                        /* 开启看门狗 */
    set_buffer( Rx2_Buffer );

    /* Infinite loop */
    while ( 1 )
    {
        Remote_Infrared_KeyDeCode();
        MX_IWDG_Refresh();
        for ( int i = 0; i < 4; i++ )
            seq[i] = 0;
        int r = rand() % 2;
        if ( r == 0 )
        {
            read_light();
            seq[0] = 1;
            if ( seq[0] == 1 )
            {
                if ( Red )
                    show_LED();
                else{
                    Tx1_Buffer[0] = 0x00; /* 清除晶体管 */
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
                update_state_and_cnt();
        }else  {
            read_light();
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
                    show_LED();
                else{
                    Tx1_Buffer[0] = 0x00; /* 清除晶体管 */
                    I2C_ZLG7290_Write( &hi2c1, 0x70, ZLG_WRITE_ADDRESS1, Tx1_Buffer, 8 );
                }
							}
        }
        HAL_Delay( cnt * 50 + 10000 );
    }
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


/* USER CODE END 4 */

void HAL_SYSTICK_Callback( void )
{
    if ( GlobalTimingDelay100us != 0 )
    {
        GlobalTimingDelay100us--;
    }
}


void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
    Remote_Infrared_KEY_ISR();
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
    case 14: /* #号键 */
        Tx1_Buffer[0] = 0x00;
        I2C_ZLG7290_Write( &hi2c1, 0x70, ZLG_WRITE_ADDRESS1, Tx1_Buffer, 8 );
        break;
    case 15:
        Tx1_Buffer[0] = 0xFC;
        break;
    default:
        break;
    }
    if ( point ) /* 是否打印小数点 */
    {
        Tx1_Buffer[0] |= 0x1;
    }
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
