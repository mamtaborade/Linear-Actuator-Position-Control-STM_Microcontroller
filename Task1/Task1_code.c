#include "stm32c0xx_hal.h"
#include <stdio.h>

// ** HAL Handles **
ADC_HandleTypeDef hadc;

// ** Pin Definitions **
#define MOTOR_LED_PIN        GPIO_PIN_5   // PA5 (Motor Reached Position Indicator)
#define VALVE_LED_PIN        GPIO_PIN_0   // PB0 (Valve Status Indicator)
#define POSITION_SENSOR_PIN  GPIO_PIN_1   // PA1 (ADC Input for Position Sensor)
#define PRESSURE_SENSOR_PIN  GPIO_PIN_2   // PA2 (ADC Input for Pressure Sensor)

// ** Thresholds for Control **
#define PRESSURE_THRESHOLD   2000   // Simulated Pressure Limit
#define POSITION_THRESHOLD   2000   // Simulated Encoder Target Value

// ** Function Prototypes **
void SystemClock_Config(void);
void MX_ADC_Init(void);
void Serial_Update(uint32_t position, uint32_t pressure);
uint32_t Read_ADC(uint32_t channel);
void MX_GPIO_Init(void);


// ** Read ADC Channel Function **
uint32_t Read_ADC(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_79CYCLES_5;

    HAL_ADC_ConfigChannel(&hadc, &sConfig);
    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
    uint32_t value = HAL_ADC_GetValue(&hadc);
    HAL_ADC_Stop(&hadc);
    return value;
}


// ** Serial Monitor Update Function **
void Serial_Update(uint32_t position, uint32_t pressure) {
    Serial.print("	Current Position : "); Serial.print(position);
    Serial.print("  Current Pressure: "); Serial.print(pressure);
    Serial.print(" 	Actuator: "); Serial.print(position >= POSITION_THRESHOLD ? "ON" : "OFF");
    Serial.print(" 	PPR Valve: "); Serial.println(pressure >= PRESSURE_THRESHOLD ? "OPEN" : "CLOSED");
}


// ** System Clock Configuration **
void SystemClock_Config(void) {
    // Default clock settings for STM32C0xx in Wokwi
}


int main(void) {
    // ** HAL Initialization **
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC_Init();

    // ** Initialize Serial Monitor **
    Serial.begin(115200);
    Serial.println("**** Actuator Position Control : STM Microcontroller ****");

    uint32_t pressure_value = 0;
    uint32_t position_value = 0;


    while (1) {
        // ** Read Sensor Values **
        position_value = Read_ADC(POSITION_SENSOR_PIN);
        pressure_value = Read_ADC(PRESSURE_SENSOR_PIN);

        // ** Display on Serial Monitor **
        Serial_Update(position_value, pressure_value);

        // ** Valve LED Control (PPR Valve) **
        if (pressure_value >= PRESSURE_THRESHOLD) {
            HAL_GPIO_WritePin(GPIOB, VALVE_LED_PIN, GPIO_PIN_SET);   // Valve OPEN
        } else {
            HAL_GPIO_WritePin(GPIOB, VALVE_LED_PIN, GPIO_PIN_RESET);   // Valve CLOSED
        }

        // ** Motor LED Control (Linear Accelerator Position) **
        if (position_value >= POSITION_THRESHOLD) {
            HAL_GPIO_WritePin(GPIOA, MOTOR_LED_PIN, GPIO_PIN_SET);   // LED ON
        } else {
            HAL_GPIO_WritePin(GPIOA, MOTOR_LED_PIN, GPIO_PIN_RESET);   // LED OFF
        }

        HAL_Delay(500);   // ** Refresh every 500ms **
    }
}


// ** ADC Initialization **
void MX_ADC_Init(void) {
    __HAL_RCC_ADC_CLK_ENABLE();
    hadc.Instance = ADC1;
    hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc.Init.ContinuousConvMode = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc);
}


// ** GPIO Initialization **
void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // ** Valve LED (PB0) **
    GPIO_InitStruct.Pin = VALVE_LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // ** Motor LED (PA5) **
    GPIO_InitStruct.Pin = MOTOR_LED_PIN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}