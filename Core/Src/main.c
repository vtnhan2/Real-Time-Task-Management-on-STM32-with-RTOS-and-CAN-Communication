/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "LCD_Driver.h"
#include <math.h>
#include "Touch.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// MADCTL options for ST7789:
#define SCREEN_W    240
#define SCREEN_H    320
#define ADC_MAX     4095

#define MADCTL_PORTRAIT        0x00
#define MADCTL_LANDSCAPE       0x60
#define MADCTL_INV_PORTRAIT    0xC0
#define MADCTL_INV_LANDSCAPE   0xA0

#define GROUP_ID "13" // define Group ID
// Button :[NEXT]
static uint16_t btnX = 80;
static uint16_t btnY = 250;
static uint16_t btnW = 80;
static uint16_t btnH = 40;
// zone : [Nhom 13]
#define GROUP_BOX_X   10
#define GROUP_BOX_Y   10
#define GROUP_BOX_W   145
#define GROUP_BOX_H   30

// Touch zone : task2-01
#define ZONE0_X1  20
#define ZONE0_Y1  50
#define ZONE_W    90
#define ZONE_H    70
// Touch zone : task2-02
#define ZONE1_X1  130
#define ZONE1_Y1  50
// Touch zone : task2-03
#define ZONE2_X1  20
#define ZONE2_Y1  140
// Touch zone : task2-04
#define ZONE3_X1  130
#define ZONE3_Y1  140

// button [BACK]
#define BACK_W   70
#define BACK_H   30
#define BACK_X   160
#define BACK_Y   280



//-------------------------------
//Debug touch
#define TOUCH_XFAC   (-0.070588f)
#define TOUCH_XOFF   (265)
#define TOUCH_YFAC   (-0.094117f)
#define TOUCH_YOFF   (355)
//DAC___TONE
#define NOTE_C4  262U
#define NOTE_D4  294U
#define NOTE_E4  330U
#define NOTE_F4  349U
#define NOTE_G4  392U
#define NOTE_A4  440U
#define NOTE_B4  494U
#define NOTE_C5  523U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;

/* Definitions for Task01_Display */
osThreadId_t Task01_DisplayHandle;
const osThreadAttr_t Task01_Display_attributes = {
  .name = "Task01_Display",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task02_Touch */
osThreadId_t Task02_TouchHandle;
const osThreadAttr_t Task02_Touch_attributes = {
  .name = "Task02_Touch",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_LED */
osThreadId_t Task_LEDHandle;
const osThreadAttr_t Task_LED_attributes = {
  .name = "Task_LED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for Task_CAN */
osThreadId_t Task_CANHandle;
const osThreadAttr_t Task_CAN_attributes = {
  .name = "Task_CAN",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for Task_DispCAN */
osThreadId_t Task_DispCANHandle;
const osThreadAttr_t Task_DispCAN_attributes = {
  .name = "Task_DispCAN",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for Task_DAC */
osThreadId_t Task_DACHandle;
const osThreadAttr_t Task_DAC_attributes = {
  .name = "Task_DAC",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* USER CODE BEGIN PV */


// ---------------------------------------------------------------------
//######################################################################

// Zone & FL AG  activation states (true=active)
static bool zoneActive[4] = {false,false,false,false};
static volatile bool prevBase = false;
static volatile bool prevMenu = false;
static volatile bool FLAG_Base = true;
static volatile bool FLAG_Menu = true;
static volatile bool FLAG_LED  = false;
static volatile bool FLAG_CAN  = false;
static volatile bool FLAG_Disp = false;
static volatile bool FLAG_DAC  = false;
static volatile bool WKUP	   = false;
static const char *zoneNames[4] = {
  "Task02-1","Task02-2","Task02-3","Task02-4"
};
static osThreadId_t zoneTaskHandles[4];
//######################################################################
// ---------------------------------------------------------------------


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_DAC_Init(void);
static void MX_UART4_Init(void);
static void MX_ADC1_Init(void);
void StartTask01_Display(void *argument);
void StartTask02_Touch(void *argument);
void StartTask_LED(void *argument);
void StartTask_CAN(void *argument);
void StartTask_DispCAN(void *argument);
void StartTask_DAC(void *argument);

/* USER CODE BEGIN PFP */
void ProcessMenuTouch(void);
void drawZones(void);
// Rotate LCD
void lcd_set_rotation(uint8_t mode)
{
    uint8_t val;
    switch(mode) {
        case 0: val = MADCTL_PORTRAIT;      break;
        case 1: val = MADCTL_LANDSCAPE;     break;
        case 2: val = MADCTL_INV_PORTRAIT;  break;
        case 3: val = MADCTL_INV_LANDSCAPE; break;
        default: val = MADCTL_PORTRAIT;
    }
    lcd_write_command(0x36, val);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// -----------LCD Display ------------------------------------------
//######################################################################
void Draw_BaseScreen(void)
{
/*
 * sketch BaseMenu
┌──────────────────────────────────────────────┐
│                                              │
│       DO AN: HE THONG NHUNG                  │  Font1608, background : WHITE
│                                              │
│             Nhom: 13                         │
│                                              │
├──────────────────────────────────────────────┤
│                                              │
│   Thanh vien:                                │
│                                              │
│   Vo Thanh Nhan   22200114                   │
│   Le Tien Thang   22200144                   │
│   Lam Minh Quan   22200131                   │
│                                              │
├──────────────────────────────────────────────┤
│                                              │
│            Press NEXT to continue...         │
│                                              │
│              ┌───────────────┐               │
│              │    NEXT       │               │
│              └───────────────┘               │
│                                              │
└──────────────────────────────────────────────┘
 * */

    lcd_clear_screen(0xFFFF);    // Clear entire screen to white
    lcd_display_string(30, 30, (uint8_t *)"DO AN: TH HE THONG NHUNG", FONT_1608, 0x001F);
    lcd_display_string(88, 56, (uint8_t *)"Nhom: 13", FONT_1608, 0x001F);
    lcd_draw_h_line(0, 82, SCREEN_W, 0x001F);
    lcd_display_string(20, 92, (uint8_t *)"Thanh vien:", FONT_1608, 0x001F);
    lcd_display_string(20, 112, (uint8_t *)"Vo Thanh Nhan   22200114", FONT_1608, 0x001F);
    lcd_display_string(20, 132, (uint8_t *)"Le Tien Thang   22200144", FONT_1608, 0x001F);
    lcd_display_string(20, 152, (uint8_t *)"Lam Minh Quan   22200131", FONT_1608, 0x001F);
    lcd_draw_h_line(0, 178, SCREEN_W, 0x001F);
    lcd_display_string(20, 200, (uint8_t *)"Nhan [NEXT] de tiep tuc...", FONT_1608, 0x001F);

    //Draw NEXT button centered at bottom
    const uint16_t btnX = 80;
    const uint16_t btnY = 250;
    const uint16_t btnW = 80;
    const uint16_t btnH = 40;
    // Draw blue border for NEXT box
    lcd_draw_rect(btnX, btnY, btnW, btnH, 0x001F);
    lcd_display_string(104, 262, (uint8_t *)"NEXT", FONT_1608, 0x001F);
}
void ShowMainMenu(void)
{
/* sketch Main Menu
┌────────────────────────────────────────┐
│            MAIN MENU                   │
│                                        │
│   ┌──────────────┐  ┌──────────────┐   │
│   │  Task 02-1   │  │  Task 02-2   │   │
│   └──────────────┘  └──────────────┘   │
│                                        │
│   ┌──────────────┐  ┌──────────────┐   │
│   │  Task 02-3   │  │  Task 02-4   │   │
│   └──────────────┘  └──────────────┘   │
│                                        │
│                          ┌──────────┐  │
│                          │   BACK   │  │
│                          └──────────┘  │
└────────────────────────────────────────┘

 * */
    // 1) Clear background
    lcd_clear_screen(BLUE);

    // 2) Draw group info box
    lcd_fill_rect(GROUP_BOX_X, GROUP_BOX_Y,
                  GROUP_BOX_W, GROUP_BOX_H,
                  WHITE);
    lcd_display_string(GROUP_BOX_X + 5,
                       GROUP_BOX_Y + 8,
                       (uint8_t *)" Nhom: [nhom 13]",
                       FONT_1608,
                       BLACK);

    // 3) Draw Zone 0 (LED task)
    if (FLAG_LED)
        lcd_fill_rect(ZONE0_X1, ZONE0_Y1, ZONE_W, ZONE_H, GREEN);
    else
        lcd_fill_rect(ZONE0_X1, ZONE0_Y1, ZONE_W, ZONE_H, WHITE);
    lcd_display_string(ZONE0_X1 + 10,
                       ZONE0_Y1 + 25,
                       (uint8_t *)"Task 02-1",
                       FONT_1608,
                       FLAG_LED ? WHITE : BLACK);

    // 4) Draw Zone 1 (CAN task)
    if (FLAG_CAN)
        lcd_fill_rect(ZONE1_X1, ZONE1_Y1, ZONE_W, ZONE_H, GREEN);
    else
        lcd_fill_rect(ZONE1_X1, ZONE1_Y1, ZONE_W, ZONE_H, WHITE);
    lcd_display_string(ZONE1_X1 + 10,
                       ZONE1_Y1 + 25,
                       (uint8_t *)"Task 02-2",
                       FONT_1608,
                       FLAG_CAN ? WHITE : BLACK);

    // 5) Draw Zone 2 (Disp CAN task)
    if (FLAG_Disp)
        lcd_fill_rect(ZONE2_X1, ZONE2_Y1, ZONE_W, ZONE_H, GREEN);
    else
        lcd_fill_rect(ZONE2_X1, ZONE2_Y1, ZONE_W, ZONE_H, WHITE);
    lcd_display_string(ZONE2_X1 + 10,
                       ZONE2_Y1 + 25,
                       (uint8_t *)"Task 02-3",
                       FONT_1608,
                       FLAG_Disp ? WHITE : BLACK);

    // 6) Draw Zone 3 (DAC task)
    if (FLAG_DAC)
        lcd_fill_rect(ZONE3_X1, ZONE3_Y1, ZONE_W, ZONE_H, GREEN);
    else
        lcd_fill_rect(ZONE3_X1, ZONE3_Y1, ZONE_W, ZONE_H, WHITE);
    lcd_display_string(ZONE3_X1 + 10,
                       ZONE3_Y1 + 25,
                       (uint8_t *)"Task 02-4",
                       FONT_1608,
                       FLAG_DAC ? WHITE : BLACK);

    // 7) Draw Back button
    lcd_fill_rect(BACK_X, BACK_Y, BACK_W, BACK_H, WHITE);
    lcd_display_string(BACK_X + 10,
                       BACK_Y + 8,
                       (uint8_t *)"[Back]",
                       FONT_1608,
                       BLACK);

    // Delay to allow other tasks to run
    osDelay(1);
}

void Draw_Task02_3_Screen(const uint8_t *buf)
{
    char text[32];
    uint8_t groupId = buf[0];
    float temperature;
    memcpy(&temperature, &buf[1], sizeof(float));
    // Clear screen to white
    lcd_clear_screen(0xFFFF);
    lcd_display_string(60, 10, (uint8_t *)"TASK 02-3", FONT_1608, 0x0000);
    lcd_draw_rect(20, 50, 200, 150, 0x001F);  // Blue border
    snprintf(text, sizeof(text), "Nhom: %02d", groupId);
    lcd_display_string(30, 75, (uint8_t *)text, FONT_1608, 0x001F);
    snprintf(text, sizeof(text), "Temp: %.1f C", temperature);
    lcd_display_string(30, 110, (uint8_t *)text, FONT_1608, 0x001F);
    lcd_display_string(40, 220, (uint8_t *)"(3 s) Returning...", FONT_1608, 0x7BEF);
}

//######################################################################
// ---------------------------------------------------------------------

// -----------Touch Processor ------------------------------------------
//######################################################################
void UpdateZoneDisplay(uint8_t idx,uint16_t x, uint16_t y, const char *label, volatile bool *flagPtr)
{
    // Set the flag based on zoneActive[idx]
    *flagPtr = zoneActive[idx];
    // Fill the zone rectangle: green if active, white if not
    lcd_fill_rect(x, y, ZONE_W, ZONE_H, zoneActive[idx] ? GREEN : WHITE);
    uint16_t fg = zoneActive[idx] ? WHITE : BLACK;
    lcd_display_string(x + 10, y + 25, (uint8_t *)label, FONT_1608, fg);
}

void toggleZone(uint8_t idx)
{
    if (idx>=4) return;
    zoneActive[idx] = !zoneActive[idx];
    switch (idx)
    {
        case 0:
            UpdateZoneDisplay(0, ZONE0_X1, ZONE0_Y1, "Task 02-1", &FLAG_LED);

            break;
        case 1:
            UpdateZoneDisplay(1, ZONE1_X1, ZONE1_Y1, "Task 02-2", &FLAG_CAN);
            break;
        case 2:
            UpdateZoneDisplay(2, ZONE2_X1, ZONE2_Y1, "Task 02-3", &FLAG_Disp);
            break;
        case 3:
            UpdateZoneDisplay(3, ZONE3_X1, ZONE3_Y1, "Task 02-4", &FLAG_DAC);
            break;
        default:
            break;
    }

    // debug UART
    char msg[64];
    int len = snprintf(msg,sizeof(msg),"%s %s\r\n",zoneNames[idx], zoneActive[idx]?"ON":"OFF");
    HAL_UART_Transmit(&huart4,(uint8_t*)msg,len,HAL_MAX_DELAY);
}
void ProcessMenuTouch(void);

//######################################################################
// ---------------------------------------------------------------------


// -----------Chip's Temperature----------------------------------------
//######################################################################
uint16_t readInternalTempRaw(void){ // Read ADC
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  uint16_t raw = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return raw;
}
float getChipTemperature(void){ //converting adc to temp
  uint16_t raw = readInternalTempRaw();
  uint16_t cal30  = * (uint16_t*)0x1FFF7A2C;
  uint16_t cal110 = * (uint16_t*)0x1FFF7A2E;
  return ((float)(raw - cal30) * 80.0f) / (float)(cal110 - cal30) + 30.0f;
}
//######################################################################
// ---------------------------------------------------------------------

// -----------CAN Protocol ---------------------------------------------
//######################################################################
uint8_t lastCanBuf[8];
int    lastCanLen;

void storeLastCan(uint8_t *buf,int len){
  memcpy(lastCanBuf,buf,len);
  lastCanLen = len;
}
void getLastCan(uint8_t *buf,int *len){
  memcpy(buf,lastCanBuf,lastCanLen);
  *len = lastCanLen;
}

uint16_t readInternalTemp(void){
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1,10);
  return HAL_ADC_GetValue(&hadc1);
}
//######################################################################
// ---------------------------------------------------------------------


//######################################################################
// ---------------------------------------------------------------------

// ------------Install the tone for the DAC-----------------------------
//######################################################################

static void DWT_Delay_Init(void)
{
    // Enable DWT counter for microsecond delays
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;
}

static void DWT_Delay_us(uint32_t microseconds)
{
    // Busy‐wait for the specified number of microseconds
    uint32_t start  = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000U) * microseconds;
    while ((DWT->CYCCNT - start) < cycles);
}

static void playToneDAC(uint16_t noteFreq, uint32_t durationMs)
{
    // Generate a square wave on DAC1 channel 2 (PA5) at the specified frequency
    if (noteFreq == 0)
    {
        // Silence if frequency is zero
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
        return;
    }

    // Calculate half‐period in microseconds
    uint32_t half_us = 500000U / noteFreq;
    uint32_t t0      = HAL_GetTick();
    uint32_t elapsed = 0;

    while (elapsed < durationMs)
    {
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
        DWT_Delay_us(half_us);

        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
        DWT_Delay_us(half_us);

        elapsed = HAL_GetTick() - t0;
        if (elapsed >= durationMs) break;
    }

    // Ensure DAC output is low at the end
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
}


static void sleepMode_init(void)
{
    HAL_SuspendTick();                         // pause HAL tick

    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // stop SysTick
    NVIC_DisableIRQ(SysTick_IRQn);

    // disable unused IRQs
    NVIC_DisableIRQ(CAN1_RX0_IRQn);
    NVIC_DisableIRQ(CAN2_RX0_IRQn);
    NVIC_DisableIRQ(DMA1_Stream1_IRQn);
    NVIC_DisableIRQ(UART4_IRQn);

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);      // clear PB4 EXTI flag

    // disable CAN interrupts
    HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_DeactivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

    // drain CAN2 FIFO0
    {
        CAN_RxHeaderTypeDef tmpHdr;
        uint8_t tmpBuf[8];
        while (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) > 0) {
            HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &tmpHdr, tmpBuf);
        }
    }

    __disable_irq();
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    __enable_irq();

    // wake-up: restore ticks and IRQs
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    NVIC_EnableIRQ(SysTick_IRQn);
    HAL_ResumeTick();

    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    NVIC_EnableIRQ(CAN2_RX0_IRQn);
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    NVIC_EnableIRQ(UART4_IRQn);

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

    WKUP = 1;
}

//######################################################################
// ---------------------------------------------------------------------
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	//uint8_t count = 0;
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
  MX_SPI1_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_DAC_Init();
  MX_UART4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  tp_init();
  tp_adjust();
  tp_dialog();
  lcd_set_rotation(2);

  FLAG_Base = true;// Ensure Base screen shows first
  FLAG_Menu = false;

  const char msg[] = "Project Start: \r\n";    // Send "project start" via UART for testing
  HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task01_Display */
  Task01_DisplayHandle = osThreadNew(StartTask01_Display, NULL, &Task01_Display_attributes);

  /* creation of Task02_Touch */
  Task02_TouchHandle = osThreadNew(StartTask02_Touch, NULL, &Task02_Touch_attributes);

  /* creation of Task_LED */
  Task_LEDHandle = osThreadNew(StartTask_LED, NULL, &Task_LED_attributes);

  /* creation of Task_CAN */
  Task_CANHandle = osThreadNew(StartTask_CAN, NULL, &Task_CAN_attributes);

  /* creation of Task_DispCAN */
  Task_DispCANHandle = osThreadNew(StartTask_DispCAN, NULL, &Task_DispCAN_attributes);

  /* creation of Task_DAC */
  Task_DACHandle = osThreadNew(StartTask_DAC, NULL, &Task_DAC_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 256;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef filt = {
    .FilterBank           = 0,
    .FilterMode           = CAN_FILTERMODE_IDMASK,
    .FilterScale          = CAN_FILTERSCALE_32BIT,
    .FilterIdHigh         = 0x0000,
    .FilterIdLow          = 0x0000,
    .FilterMaskIdHigh     = 0x0000,
    .FilterMaskIdLow      = 0x0000,
    .FilterFIFOAssignment = CAN_FILTER_FIFO0,
    .FilterActivation     = CAN_FILTER_ENABLE,
  };
  if (HAL_CAN_ConfigFilter(&hcan1, &filt) != HAL_OK) {
    Error_Handler();
  }

  // 2) Bắt đầu CAN1
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    Error_Handler();
  }

  // 3) Kích hoạt interrupt khi có message pending
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  CAN_FilterTypeDef filt = {
    .FilterBank           = 14,
    .FilterMode           = CAN_FILTERMODE_IDMASK,
    .FilterScale          = CAN_FILTERSCALE_32BIT,
    .FilterIdHigh         = 0x0000,
    .FilterIdLow          = 0x0000,
    .FilterMaskIdHigh     = 0x0000,
    .FilterMaskIdLow      = 0x0000,
    .FilterFIFOAssignment = CAN_FILTER_FIFO0,
    .FilterActivation     = CAN_FILTER_ENABLE,
  };
  if (HAL_CAN_ConfigFilter(&hcan2, &filt) != HAL_OK) {
    Error_Handler();
  }

  // 3) Bắt đầu CAN2
  if (HAL_CAN_Start(&hcan2) != HAL_OK) {
    Error_Handler();
  }

  // 4) Kích hoạt interrupt để callback khi có message pending
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|LCD_CS_Pin|LCD_RS_Pin|TP_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_Pin LCD_BL_Pin */
  GPIO_InitStruct.Pin = LED_Pin|LCD_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_IRQ_Pin */
  GPIO_InitStruct.Pin = TP_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TP_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_CS_Pin */
  GPIO_InitStruct.Pin = TP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TP_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_4);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN2)
    {
        CAN_RxHeaderTypeDef rxHdr;
        uint8_t rxData[8];
        char uartBuf[64];
        float tempC;
        uint8_t groupId;
        int msgLen;

        // Fetch the received CAN message from FIFO0
        if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rxHdr, rxData) != HAL_OK)
        {
            return; // error reading
        }

        //  Only accept frames with StdId = 0x321 and DLC >= 5
        if ((rxHdr.StdId == 0x321) && (rxHdr.DLC >= 5))
        {
            //  Store 5 bytes (group + float) for Task02-3
            storeLastCan(rxData, 5);

            // Parse group ID (1st byte) and temperature (next 4 bytes)
            groupId = rxData[0];
            memcpy(&tempC, &rxData[1], sizeof(float));

            //  Send formatted string over UART4
            msgLen = snprintf(uartBuf, sizeof(uartBuf),
                              "Nhom: %02d, Temp: %.1f C\r\n",
                              groupId, tempC);
            HAL_UART_Transmit(&huart4, (uint8_t *)uartBuf, msgLen, HAL_MAX_DELAY);
        }
    }
}

static void ScaleTouch(uint16_t raw_x, uint16_t raw_y,
                       uint16_t *px,  uint16_t *py)
{
    *px =  raw_y* (SCREEN_W  - 1) / ADC_MAX;
    *py = raw_x * (SCREEN_H - 1) / ADC_MAX;
}

void ProcessMenuTouch(void)
{
    static bool lock = false;
    uint16_t raw_x, raw_y, x, y;

    if (tp_scan(0))
    {
        xpt2046_read_xy(&raw_x, &raw_y);
        ScaleTouch(raw_x, raw_y, &x, &y);

        if (!lock)
        {
            lock = true;

            // If Base screen is showing, check for NEXT button tap
            if (FLAG_Base)
            {
                if (x >= btnX && x < btnX + btnW &&
                    y >= btnY && y < btnY + btnH)
                {
                    FLAG_Base = false;
                    FLAG_Menu = true;
                    // The Display task will notice FLAG_Menu and draw Main menu.
                    return;
                }
            }
            else if(WKUP){
                toggleZone(3);
            	WKUP = 0;
            }
            // If Main menu is showing, check for BACK button tap
            else if (FLAG_Menu)
            {
                if (x >= BACK_X && x < BACK_X + BACK_W &&
                    y >= BACK_Y && y < BACK_Y + BACK_H)
                {
                    FLAG_Menu = false;
                    FLAG_Base = true;
                    // The Display task will notice FLAG_Base and redraw Base screen.
                    return;
                }

                //Otherwise still in Main menu: check the four Task-zones
                if (x >= ZONE0_X1 && x < ZONE0_X1 + ZONE_W &&
                    y >= ZONE0_Y1 && y < ZONE0_Y1 + ZONE_H)
                {
                    toggleZone(0);
                }
                else if (x >= ZONE1_X1 && x < ZONE1_X1 + ZONE_W &&
                         y >= ZONE1_Y1 && y < ZONE1_Y1 + ZONE_H)
                {
                    toggleZone(1);
                }
                else if (x >= ZONE2_X1 && x < ZONE2_X1 + ZONE_W &&
                         y >= ZONE2_Y1 && y < ZONE2_Y1 + ZONE_H)
                {
                    toggleZone(2);
                }
                else if (x >= ZONE3_X1 && x < ZONE3_X1 + ZONE_W &&
                         y >= ZONE3_Y1 && y < ZONE3_Y1 + ZONE_H)
                {
                    toggleZone(3);
                }
            }
        }
    }
    else
    {
        lock = false;
    }
}
//static void DebugTouch(void)
//{
//    uint16_t raw_x, raw_y;
//    char buf[80];
//
//    // Nếu đang chạm
//    if (tp_scan(0))
//    {
//        // 1) Đọc giá trị ADC thô
//        xpt2046_read_xy(&raw_x, &raw_y);
//
//        // 2) Tính tọa độ đã hiệu chỉnh
//        int16_t cal_x = (int16_t)(TOUCH_XFAC * raw_y + TOUCH_XOFF);
//        int16_t cal_y = (int16_t)(TOUCH_YFAC * raw_x + TOUCH_YOFF);
//
//        // 3) In ra UART: raw và calibrated
//        int len = snprintf(buf, sizeof(buf),
//            "rawX=%u rawY=%u => calX=%d calY=%d\r\n",
//            raw_x, raw_y, cal_x, cal_y);
//        HAL_UART_Transmit(&huart4, (uint8_t*)buf, len, HAL_MAX_DELAY);
//
//        // Chờ chút để nhìn rõ trên terminal
//        osDelay(200);
//    }
//}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask01_Display */
/**
  * @brief  Function implementing the Task01_Display thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask01_Display */
void StartTask01_Display(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	const char msg[] = "StartTask01_Display \r\n";
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    FLAG_Menu = false;
    FLAG_Base = true;

    while (1)
    {
        // If Base-screen flag just got set, draw Base screen once
        if (FLAG_Base && !prevBase)
        {
            Draw_BaseScreen();
            prevBase = true;
            prevMenu = false;
        }

        // If Main-menu flag just got set, draw Main menu once
        if (FLAG_Menu && !prevMenu)
        {
			FLAG_LED  = false;
			FLAG_CAN  = false;
			FLAG_Disp = false;
			FLAG_DAC  = false;
			zoneActive[0] = 0;
			zoneActive[1] = 0;
			zoneActive[2] = 0;
			zoneActive[3] = 0;
            ShowMainMenu();
            prevMenu = true;
            prevBase = false;
        }

        // Otherwise, do nothing except yield
        osDelay(50);
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02_Touch */
/**
* @brief Function implementing the Task02_Touch thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02_Touch */
void StartTask02_Touch(void *argument)
{
  /* USER CODE BEGIN StartTask02_Touch */
  /* Infinite loop */
	zoneTaskHandles[0] = Task_LEDHandle;
	zoneTaskHandles[1] = Task_CANHandle;
	zoneTaskHandles[2] = Task_DispCANHandle;
	zoneTaskHandles[3] = Task_DACHandle;
	const char msg[] = "StartTask02_Touch \r\n";
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
  for(;;)
  {
	    ProcessMenuTouch();
	    //DebugTouch();
	    osDelay(50);
  }
  /* USER CODE END StartTask02_Touch */
}

/* USER CODE BEGIN Header_StartTask_LED */
/**
* @brief Function implementing the Task_LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_LED */
void StartTask_LED(void *argument)
{
  /* USER CODE BEGIN StartTask_LED */
  /* Infinite loop */
	  for(;;)
	  {
	    if (FLAG_LED) {
	      HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	      osDelay(1000);
	    } else {
	    	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin, 0);
	      osDelay(100);
	    }
	  }
  /* USER CODE END StartTask_LED */
}

/* USER CODE BEGIN Header_StartTask_CAN */
/**
* @brief Function implementing the Task_CAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_CAN */
void StartTask_CAN(void *argument)
{
  /* USER CODE BEGIN StartTask_CAN */
  /* Infinite loop */
    CAN_TxHeaderTypeDef txHdr = {
        .StdId             = 0x321,
        .IDE               = CAN_ID_STD,
        .RTR               = CAN_RTR_DATA,
        .DLC               = 5,
        .TransmitGlobalTime = DISABLE
    };
    uint8_t txData[8];
    uint32_t txMailbox;
    uint8_t groupId = (uint8_t)atoi(GROUP_ID);
    for (;;)
    {
        if (FLAG_CAN)
        {
            // Read internal temperature as float
            float tempC = getChipTemperature();

            //  Pack group ID (1 byte) + temperature (4-byte IEEE-754)
            txData[0] = groupId;
            memcpy(&txData[1], &tempC, sizeof(float));

            txData[5] = txData[6] = txData[7] = 0;

            // Transmit on CAN1
            if (HAL_CAN_AddTxMessage(&hcan1, &txHdr, txData, &txMailbox) != HAL_OK)
            {
            }
            osDelay(500);
        }
        else
        {
            osDelay(100);
        }
    }
  /* USER CODE END StartTask_CAN */
}

/* USER CODE BEGIN Header_StartTask_DispCAN */
/**
* @brief Function implementing the Task_DispCAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_DispCAN */
void StartTask_DispCAN(void *argument)
{
  /* USER CODE BEGIN StartTask_DispCAN */
  /* Infinite loop */
    uint8_t buf[8]; int len;

    for (;;)
    {
        if (FLAG_Disp)
        {
            //  Fetch the stored CAN frame (group + temperature)
            getLastCan(buf, &len);
            Draw_Task02_3_Screen(buf);
            osDelay(3000);
            FLAG_Disp = false;
            prevMenu = false;

            // debug UART
            char msg[64];
            int len = snprintf(msg,sizeof(msg),"%s %s\r\n","Task02-3", "OFF");
            HAL_UART_Transmit(&huart4,(uint8_t*)msg,len,HAL_MAX_DELAY);
        }
        else
        {
            osDelay(100);
        }
    }
  /* USER CODE END StartTask_DispCAN */
}

/* USER CODE BEGIN Header_StartTask_DAC */
/**
* @brief Function implementing the Task_DAC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_DAC */
void StartTask_DAC(void *argument)
{
  /* USER CODE BEGIN StartTask_DAC */
  /* Infinite loop */
    uint32_t startTick, elapsed, remaining;
    uint8_t  noteIndex;
    char     uartBuf[64];

    static const uint16_t melody[] = {
        NOTE_C5, NOTE_B4, NOTE_A4, NOTE_G4,
        NOTE_F4, NOTE_E4, NOTE_D4, NOTE_C4
    };
    const uint8_t  MELODY_LEN = sizeof(melody) / sizeof(melody[0]);
    const uint32_t TOTAL_MS   = 10000U;  // 10 seconds
    const uint32_t NINE_MS    = 9000U;   // 9 seconds

    DWT_Delay_Init();
    HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

    for (;;)
    {
        if (FLAG_DAC)
        {
            int len = snprintf(uartBuf, sizeof(uartBuf),
                               "Task02_4: Start alarm 10s\r\n");
            HAL_UART_Transmit(&huart4, (uint8_t *)uartBuf, len, HAL_MAX_DELAY);

            startTick = HAL_GetTick();
            elapsed   = 0;
            noteIndex = 0;

            while (elapsed < TOTAL_MS)
            {
                elapsed = HAL_GetTick() - startTick;
                if (elapsed >= TOTAL_MS) break;
                remaining = TOTAL_MS - elapsed;

                if (remaining <= (TOTAL_MS - NINE_MS))
                {

                    len = snprintf(uartBuf, sizeof(uartBuf),
                                   "Task02_4: Remaining %4lu ms\r\n", remaining);
                    HAL_UART_Transmit(&huart4, (uint8_t *)uartBuf, len, HAL_MAX_DELAY);

                    playToneDAC(NOTE_C4, remaining);
                    break;
                }
                else
                {
                    len = snprintf(uartBuf, sizeof(uartBuf),
                                   "Task02_4: Remaining %4lu ms\r\n", remaining);
                    HAL_UART_Transmit(&huart4, (uint8_t *)uartBuf, len, HAL_MAX_DELAY);

                    playToneDAC(melody[noteIndex], 500U);

                    elapsed = HAL_GetTick() - startTick;
                    if (elapsed >= TOTAL_MS) break;

                    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
                    {
                        uint32_t t1 = HAL_GetTick();
                        while ((HAL_GetTick() - t1) < 50U) { }
                    }

                    noteIndex = (noteIndex + 1U) % MELODY_LEN;
                    elapsed   = HAL_GetTick() - startTick;
                }
            }

            HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

            len = snprintf(uartBuf, sizeof(uartBuf),
                           "Task02_4: --> SleepMode\r\n");
            HAL_UART_Transmit(&huart4, (uint8_t *)uartBuf, len, HAL_MAX_DELAY);

            sleepMode_init();

            len = snprintf(uartBuf, sizeof(uartBuf),
                           "Task02_4: WOKE UP !\r\n");
            HAL_UART_Transmit(&huart4, (uint8_t *)uartBuf, len, HAL_MAX_DELAY);

            FLAG_DAC = false;

        }
        else
        {
            osDelay(100);
        }
    }
  /* USER CODE END StartTask_DAC */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
