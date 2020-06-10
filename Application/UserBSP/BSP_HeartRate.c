#include "BSP_HeartRate.h"

I2C_HandleTypeDef hi2c1;
MAX30102_HandleTypeDef hMAX;

void heartRate_init( void ) {
    i2c_init();

    //  realSampleRate = sampleRate / sampleAverage
    hMAX.sampleRate = MAX30102_CONF_SAMPLE_RATE_100;
    hMAX.sampleAverage = MAX30102_CONF_SAMPLE_AVERAGE_4;

    hMAX.pulseWidth = MAX30102_CONF_LED_PULSE_WIDTH_411;
    hMAX.adcRange = MAX30102_CONF_ADC_RANGE_2048;

    hMAX.fifoRollover = MAX30102_CONF_FIFO_ROLLOVER_EN;
    hMAX.fifoAlmostFull = 0xFU;

    hMAX.mode = MAX30102_CONF_MODE_SpO;

    hMAX.pulseAmplitudeRed = 0xFFU;
    hMAX.pulseAmplitudeIR = 0xFFU;

    hMAX.multiLedSlot1 = MAX30102_CONF_LED_SLOT_NONE;
    hMAX.multiLedSlot2 = MAX30102_CONF_LED_SLOT_NONE;
    hMAX.multiLedSlot3 = MAX30102_CONF_LED_SLOT_NONE;
    hMAX.multiLedSlot4 = MAX30102_CONF_LED_SLOT_NONE;

    MAX30102_init(&hi2c1, &hMAX);
}

void i2c_init( void ) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

int32_t heartRate_getHeartRate( void ) {
    return MAX30102_getHeartRate();
}