/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  * Bu program STM32WB55RG üzerinde çalışır.
  *
  * Özellikler:
  * - PD0 buton ile mod değiştirilebilir (debounce ile). ÇALIŞAN KOD V2
  * - Mode 0: LED2 her saniye yanıp söner.
  * - Mode 1: LED3 bir kere yanar, LED2 RNG ile belirlenen sayıda yanıp söner,
  *           LED1 ise işin bittiğini belirtir.
  * - LED3 her mod değişiminde yanıp söner.
  * - UART1 üzerinden mesaj gönderilir.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stm32wbxx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */
#define LED1_PIN        GPIO_PIN_5    /* PB5 */
#define LED2_PIN        GPIO_PIN_0    /* PB0 */
#define LED3_PIN        GPIO_PIN_1    /* PB1 */
#define LED_GPIO_PORT   GPIOB

#define BUTTON_PIN      GPIO_PIN_0    /* PD0 */
#define BUTTON_PORT     GPIOD
#define BUTTON_DEBOUNCE_MS 50U

#define MODE0_BLINK_MS      1000U
#define RNG_LED_ON_MS       50U
#define RNG_LED_OFF_MS      50U
#define RNG_POST_WAIT_MS    1000U
#define LED3_ON_MS          500U
#define LED3_TOGGLE_MS      200U
/* USER CODE END PD */

/* USER CODE BEGIN ET */
typedef enum
{
    M2_IDLE = 0,
    M2_LED3_ON,
    M2_BLINKING,
    M2_LED1_ON,
    M2_WAIT
} mode2_state_t;
/* USER CODE END ET */

/* Private variables ---------------------------------------------------------*/
RNG_HandleTypeDef hrng;
UART_HandleTypeDef huart1;
/* USER CODE BEGIN PV */
volatile uint8_t mode = 0;           /* 0 = Mode0, 1 = Mode1 */
volatile uint8_t debugToggle = 0;
volatile int8_t debugForceMode = -1;

static uint8_t lastButtonReading = 1;
static uint8_t lastButtonStable = 1;
static uint32_t lastButtonChangeTime = 0;

static uint32_t m0_lastToggle = 0;
static uint8_t m0_ledState = 0;

static mode2_state_t m2_state = M2_IDLE;
static uint32_t m2_nextAction = 0;
static uint32_t m2_blinksRemain = 0;
static uint8_t m2_ledOn = 0;

static uint32_t m2_led3EndTime = 0; // LED3 kontrolü için
static uint32_t m2_modeChangeEndTime = 0; // mod değişimi sonrası LED3

static char uartMsg[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RNG_Init(void);
static void MX_USART1_UART_Init(void);

static inline uint32_t now_ms(void);
static bool button_pressed_event(void);
static void mode2_reset_state(void);
static void toggle_mode(void);
static void force_set_mode(uint8_t m);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static inline uint32_t now_ms(void)
{
    return HAL_GetTick();
}

static bool button_pressed_event(void)
{
    uint32_t t = now_ms();
    uint8_t reading = (uint8_t)HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);

    if(reading != lastButtonReading)
    {
        lastButtonReading = reading;
        lastButtonChangeTime = t;
    }

    if((t - lastButtonChangeTime) > BUTTON_DEBOUNCE_MS)
    {
        if(lastButtonStable != lastButtonReading)
        {
            lastButtonStable = lastButtonReading;
            if(lastButtonStable == GPIO_PIN_RESET)
                return true;
        }
    }
    return false;
}

static void mode2_reset_state(void)
{
    m2_state = M2_IDLE;
    m2_nextAction = 0;
    m2_blinksRemain = 0;
    m2_ledOn = 0;
    m2_led3EndTime = 0;

    HAL_GPIO_WritePin(LED_GPIO_PORT, LED2_PIN | LED1_PIN | LED3_PIN, GPIO_PIN_RESET);
}

static void toggle_mode(void)
{
    mode ^= 1u; // Toggle 0 <-> 1
    mode2_reset_state();
    m0_lastToggle = now_ms();
    m0_ledState = 0;
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);

    // LED3 mod değişiminde yanıp sönsün
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED3_PIN, GPIO_PIN_SET);
    m2_modeChangeEndTime = now_ms() + LED3_ON_MS;

    int len = snprintf(uartMsg, sizeof(uartMsg), "Mode toggled -> %u\r\n", (unsigned)mode);
    if(len>0) HAL_UART_Transmit(&huart1, (uint8_t*)uartMsg, (uint16_t)len, HAL_MAX_DELAY);
}

static void force_set_mode(uint8_t m)
{
    if(m > 1) return;
    mode = m;
    mode2_reset_state();
    m0_lastToggle = now_ms();
    m0_ledState = 0;
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);

    // LED3 mod değişiminde yanıp sönsün
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED3_PIN, GPIO_PIN_SET);
    m2_modeChangeEndTime = now_ms() + LED3_ON_MS;

    int len = snprintf(uartMsg, sizeof(uartMsg), "Mode forced -> %u\r\n", (unsigned)mode);
    if(len>0) HAL_UART_Transmit(&huart1, (uint8_t*)uartMsg, (uint16_t)len, HAL_MAX_DELAY);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_RNG_Init();
    MX_USART1_UART_Init();

    while(1)
    {
        if(button_pressed_event() || debugToggle)
        {
            toggle_mode();
            debugToggle = 0;
        }

        if(debugForceMode != -1)
        {
            force_set_mode((uint8_t)debugForceMode);
            debugForceMode = -1;
        }

        uint32_t t = now_ms();

        // Mod değişimi LED3 kontrolü
        if(m2_modeChangeEndTime && t >= m2_modeChangeEndTime)
        {
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED3_PIN, GPIO_PIN_RESET);
            m2_modeChangeEndTime = 0;
        }

        if(mode == 0) // Mode 0: LED2 yanıp söner
        {
            if(t - m0_lastToggle >= MODE0_BLINK_MS)
            {
                m0_lastToggle = t;
                m0_ledState ^= 1;
                HAL_GPIO_WritePin(LED_GPIO_PORT, LED2_PIN, m0_ledState?GPIO_PIN_SET:GPIO_PIN_RESET);
            }
        }
        else // Mode 1: RNG blink
        {
            switch(m2_state)
            {
                case M2_IDLE:
                {
                    // LED3 Mode 1 başında yanacak
                    HAL_GPIO_WritePin(LED_GPIO_PORT, LED3_PIN, GPIO_PIN_SET);
                    m2_led3EndTime = t + LED3_ON_MS;
                    m2_state = M2_LED3_ON;
                }
                break;

                case M2_LED3_ON:
                    if(t >= m2_led3EndTime)
                    {
                        HAL_GPIO_WritePin(LED_GPIO_PORT, LED3_PIN, GPIO_PIN_RESET);
                        uint32_t rnd;
                        if(HAL_RNG_GenerateRandomNumber(&hrng, &rnd) != HAL_OK)
                        {
                            HAL_GPIO_WritePin(LED_GPIO_PORT, LED3_PIN, GPIO_PIN_SET);
                            m2_led3EndTime = t + LED3_ON_MS;
                            break;
                        }
                        m2_blinksRemain = (rnd % 10) + 1;
                        m2_state = M2_BLINKING;
                        m2_ledOn = 0;
                        m2_nextAction = t;

                        snprintf(uartMsg, sizeof(uartMsg), "Random Number: %lu, LED2 blinks: %lu\r\n",
                                 rnd, m2_blinksRemain);
                        HAL_UART_Transmit(&huart1, (uint8_t*)uartMsg, strlen(uartMsg), HAL_MAX_DELAY);
                    }
                    break;

                case M2_BLINKING:
                    if(t >= m2_nextAction)
                    {
                        m2_ledOn ^= 1;
                        HAL_GPIO_WritePin(LED_GPIO_PORT, LED2_PIN, m2_ledOn?GPIO_PIN_SET:GPIO_PIN_RESET);
                        if(!m2_ledOn)
                        {
                            m2_blinksRemain--;
                            if(m2_blinksRemain == 0)
                            {
                                m2_state = M2_LED1_ON;
                                m2_nextAction = t + RNG_POST_WAIT_MS;
                                HAL_GPIO_WritePin(LED_GPIO_PORT, LED1_PIN, GPIO_PIN_SET);
                                break;
                            }
                        }
                        m2_nextAction = t + (m2_ledOn?RNG_LED_ON_MS:RNG_LED_OFF_MS);
                    }
                    break;

                case M2_LED1_ON:
                    if(t >= m2_nextAction)
                    {
                        HAL_GPIO_WritePin(LED_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET);
                        m2_state = M2_WAIT;
                        m2_nextAction = t + RNG_POST_WAIT_MS;
                    }
                    break;

                case M2_WAIT:
                    if(t >= m2_nextAction)
                        m2_state = M2_IDLE;
                    break;
            }
        }
    }
}


/* GPIO Initialization Function */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO port clock enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* LED pins */
    GPIO_InitStruct.Pin = LED1_PIN|LED2_PIN|LED3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);

    /* Button pin */
    GPIO_InitStruct.Pin = BUTTON_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // <-- Pull-up ekledik
    HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);

    /* UART1 pins PB6/PB7 alternate function setup */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* RNG Initialization Function */
static void MX_RNG_Init(void)
{
    hrng.Instance = RNG;
    hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
    if(HAL_RNG_Init(&hrng) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USART1 Initialization Function */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 1200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI1 | RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                  RCC_CLOCKTYPE_HCLK2 | RCC_CLOCKTYPE_HCLK4;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

/* Error Handler */
void Error_Handler(void)
{
    __disable_irq();
    while(1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
