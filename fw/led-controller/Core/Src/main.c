/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "util.h"
#include "animation.h"
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define BLOCK_SIZE 32
#define N_LEDS (N_LEDS_PER_SIDE*2)


static led_t leds_a[N_LEDS];
static led_t leds_b[N_LEDS];

static led_t * volatile leds_wr = leds_a;
static led_t * volatile leds_rd = leds_b;

static uint8_t pwmData[BLOCK_SIZE*2*24] = {0};

static uint16_t led_current = 0;
static void xfer(uint16_t offset)
{
  HAL_GPIO_WritePin(ALT_GPIO_Port, ALT_Pin, GPIO_PIN_SET);
  for(uint8_t i = 0; i < BLOCK_SIZE; i++)
  {
    if(led_current < N_LEDS) {
      led_t l = leds_rd[led_current];
      led_current++;
      static const uint8_t pwm0 = 19;
      static const uint8_t pwm1 = 38;
      
      
#define Xs X(0);X(1);X(2);X(3);X(4);X(5);X(6);X(7);

#define Xc(x, o, c) pwmData[offset + o*8 + x] = (l.c & (1<<(7-x))) ? pwm1:pwm0

#define X(x) Xc(x, 0, g)
      Xs
#undef X

#define X(x) Xc(x, 1, r)
      Xs
#undef X

#define X(x) Xc(x, 2, b)
      Xs
#undef X

#undef Xs
#undef Xc

      
      
      offset += 24;
      
      /*for(uint8_t b = 0; b < 8; b++)
      {
        if(l.g & 0x80)
          pwmData[offset++] = pwm1;
        else
          pwmData[offset++] = pwm0;
        l.g <<= 1;
      }
      
      for(uint8_t b = 0; b < 8; b++)
      {
        if(l.r & 0x80)
          pwmData[offset++] = pwm1;
        else
          pwmData[offset++] = pwm0;
        l.r <<= 1;
      }
      for(uint8_t b = 0; b < 8; b++)
      {
        if(l.b & 0x80)
          pwmData[offset++] = pwm1;
        else
          pwmData[offset++] = pwm0;
        l.b <<= 1;
      }*/
      
    }
    else 
    {
      if(led_current >= N_LEDS + 2) {
        led_current = 0;
        if(leds_rd == leds_a) {
          leds_rd = leds_b;
          leds_wr = leds_a;
        }
        else
        {
          leds_rd = leds_a;
          leds_wr = leds_b;
        }
      }
      else
        led_current++;
      for(uint8_t j = 0; j<24; j++)
      {
        pwmData[offset++] = 0;
      }
    }
    
  }
  HAL_GPIO_WritePin(ALT_GPIO_Port, ALT_Pin, GPIO_PIN_RESET);
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
  xfer(0);
  
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  xfer(BLOCK_SIZE*24);
  //
}

/*void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	datasentflag=1;
}*/

static volatile uint32_t global_brightness = 100;

static led_t scale_brightness(led_t v) {
  led_t r = {v.r*global_brightness/256, v.g*global_brightness/256, v.b*global_brightness/256};
  return r;
}

void led_set_lr(led_t *buf, uint16_t i, led_t v)
{
  led_t vs = scale_brightness(v);
  buf[N_LEDS_PER_SIDE-1-i] = vs;
  buf[N_LEDS_PER_SIDE+i] = vs;  
}

void led_set_r(led_t *buf, uint16_t i, led_t v)
{
  led_t vs = scale_brightness(v);
  buf[N_LEDS_PER_SIDE-1-i] = vs;
}

void led_set_l(led_t *buf, uint16_t i, led_t v)
{
  led_t vs = scale_brightness(v);
  buf[N_LEDS_PER_SIDE+i] = vs;  
}

static void apply_foldback(led_t *buf)
{
  uint32_t sum = 0; // in units of 17.5uA
  for(uint16_t i = 0; i < N_LEDS; i++) {
    sum += buf[i].r;
    sum += buf[i].g;
    sum += buf[i].b;
  }
  
  static const uint32_t imax = 124240;
  if(sum > imax) {
    // sum/imax
    for(uint16_t i = 0; i < N_LEDS; i++) {
      buf[i].r = buf[i].r * imax / sum;
      buf[i].g = buf[i].g * imax / sum;
      buf[i].b = buf[i].b * imax / sum;
    }
  }
  
}

volatile uint32_t speed = 500;
volatile uint32_t pattern_next = 4;
volatile uint8_t speed_override = 0;
uint32_t step_inc = 125;

static void update_step_inc(uint8_t speed_mul)
{
  step_inc = 125 * 256 / (speed_mul+1);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
  
  
  uint16_t pg_counter = 0;
  for(uint16_t i = 0; i<500; i++) {
    HAL_Delay(10);
    
    // power is good
    if(HAL_GPIO_ReadPin(USB_PG_GPIO_Port, USB_PG_Pin) == GPIO_PIN_RESET) {
      pg_counter++;
    }
    else {
      pg_counter = 0;
    }
    if(pg_counter > 200) {
      break;
    }
  }
  
  if(!pg_counter) {
    while(1) {
    HAL_GPIO_TogglePin(LED_USB_GPIO_Port, LED_USB_Pin);
    HAL_Delay(100);
    }
  }
  
  HAL_GPIO_WritePin(LED_USB_GPIO_Port, LED_USB_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(HEN_5V_GPIO_Port, HEN_5V_Pin, GPIO_PIN_SET);
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  /*for(uint16_t i = 0; i<MAX_LED; i++) {
    for(uint8_t j = 0; j<24; j++) {
      pwmData[i*24 + j] = 20;
    }
  }
  pwmData[0] = 40;*/
  
  
  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)pwmData, ARRAY_SIZE(pwmData));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  led_t *wrbuf=0;
  int32_t mm18_acc=0;
  int32_t local_speed = 0;
  int32_t local_mm18_acc=0;
  animation_set_pattern(0);
  while (1)
  {
    uint8_t uart_rxbuf;
    if(HAL_UART_Receive(&huart2, (uint8_t*)&uart_rxbuf, 1, 0) == HAL_OK)
    {
      if(uart_rxbuf >= 0x80) {
        static uint8_t counts_last = 0;
        uint8_t counts = (uart_rxbuf - 0x80);
        uint8_t delta_counts = (counts-counts_last)&0x7f;
        counts_last = counts;
        uint16_t delta_mm = delta_counts * 5;
        mm18_acc += delta_mm * 18;
      }
      else {
        static uint8_t rx_param = 0xff;
        static uint8_t rx_nibble = 0;
        static uint8_t rx_value = 0;
        if(uart_rxbuf >= 'A' && uart_rxbuf <= 'Z') {
          rx_param = uart_rxbuf - 'A';
          rx_nibble = 0;
        }
        else if(isxdigit(uart_rxbuf)) {
          uint8_t v=0xff;
          if(uart_rxbuf >= '0' && uart_rxbuf <= '9')
            v = uart_rxbuf - '0';
          else if(uart_rxbuf >= 'a' && uart_rxbuf <= 'f')
            v = uart_rxbuf - 'a' + 10;
          if(v != 0xff) {
            if(rx_nibble == 0) {
              rx_value = v << 4;
              rx_nibble = 1;
            }
            else if(rx_nibble == 1) {
              rx_nibble = 2;
              rx_value |= v;
              switch(rx_param) {
                case 0: //animation
                  animation_set_pattern(rx_value);
                  break;
                case 1: //brighness
                  global_brightness = rx_value;
                  break;
                case 2: //speed override
                  speed_override = rx_value;
                break;
                case 3: //speed mul
                  update_step_inc(rx_value);
                break;
              }
            }
          }
        }
        
      }
    }
    uint32_t ticks = HAL_GetTick();
    /*{
      static uint32_t ticks_last;
      
      if(ticks-ticks_last > speed)
      {
        mm18_acc += 5*18;
        ticks_last = ticks;
      }
    }*/
    if(wrbuf != leds_wr) {
      wrbuf = leds_wr;
      
      local_speed = mm18_acc / 16;
      mm18_acc -= local_speed;
      local_mm18_acc += local_speed;
      
      static uint8_t idle_counter = 0;
      uint8_t steps = 0;
      while(local_mm18_acc >= step_inc) {
        local_mm18_acc -= step_inc;
        steps++;
        idle_counter = 10;
      }
      {
        static uint32_t ticks_last = 0;
         if(ticks-ticks_last > 1000) {
           ticks_last = ticks;
           if(idle_counter)
             idle_counter--;
        }
        
      }
      
      if(idle_counter==0){
        static uint32_t ticks_last;
        if(ticks-ticks_last > 50)
        {
          steps = 1;
          ticks_last = ticks;
        }
      }
      if(speed_override) {
        steps = 0;
        static uint16_t speed_acc = 0;
        speed_acc += speed_override;
        while(speed_acc >= 256) {
          speed_acc -= 25;
          steps++;
        }
      }
      /*for(uint16_t i = 0; i < N_LEDS_PER_SIDE; i++) {
        led_t v = {0xff, 0xff, 0xff};
        led_set_lr(wrbuf, i, v);
      }*/
      
      {
        static uint8_t pattern_next_last;
        if(pattern_next != pattern_next_last) {
          animation_set_pattern(pattern_next);
          pattern_next_last = pattern_next;
        }   
      }
      animation_render(wrbuf, steps);
      apply_foldback(wrbuf);
#if 0
      static uint16_t a = 0;
      uint16_t m = 13;
      if(steps) {
        while(steps--) {
          a++;
          if(a == m)
            a= 0;
        }
      }
      for(uint16_t i = 0; i < N_LEDS_PER_SIDE; i++) {
         led_t v= {0,0,0};
         uint16_t x = i %m;
         x = (x+(m-1-a))%m;
         
         //if(((i+(N_LEDS_PER_SIDE-1-a)) % 48) < 5) {
         if(x < 1) {
          v.r = (1)? 0xff:0;
          //v.g = (1)? 0x2f:0;
          //v.b = (1)? 0x2f:0;
         }
         
          led_set_lr(wrbuf, i, v);
      }
      /*for(uint16_t i = 0; i < N_LEDS; i++) {
        if(a==i) {
          wrbuf[i].r = 0xff;
          wrbuf[i].g = 0xff;
          wrbuf[i].b = 0xff;
        }
        else {
          wrbuf[i].r = 0;
          wrbuf[i].g = 0;
          wrbuf[i].b = 0;
        }*/
      //}
#endif
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 59;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1024;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HEN_5V_GPIO_Port, HEN_5V_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ALT_Pin|LED_USB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HEN_5V_Pin */
  GPIO_InitStruct.Pin = HEN_5V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HEN_5V_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ALT_Pin LED_USB_Pin */
  GPIO_InitStruct.Pin = ALT_Pin|LED_USB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PG_Pin */
  GPIO_InitStruct.Pin = USB_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USB_PG_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
