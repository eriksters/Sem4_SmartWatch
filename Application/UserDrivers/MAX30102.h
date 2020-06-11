#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <stdlib.h>

/********************************/
/*    Compile time configs      */
/********************************/
#define MAX30102_SAMPLE_BUFFER_SIZE 300

/********************************/
/*      I2C addresses           */
/********************************/

#define MAX30102_WRITE_ADDR 	                    0xAEU
#define MAX30102_READ_ADDR 		                    0xAFU


/********************************/
/*      Register addresses      */
/********************************/

//  Status
#define MAX30102_IT_STATUS1_REG_ADDR                0x00U
#define MAX30102_IT_STATUS2_REG_ADDR                0x01U
#define MAX30102_IT_ENABLE1_REG_ADDR                0x02U
#define MAX30102_IT_ENABLE2_REG_ADDR                0x03U

#define MAX30102_FIFO_WR_PTR_REG_ADDR               0x04U
#define MAX30102_OFV_COUNTER_REG_ADDR               0x05U
#define MAX30102_FIFO_RD_PTR_REG_ADDR               0x06U
#define MAX30102_FIFO_DATA_REG_ADDR                 0x07U

//  Configuration
#define MAX30102_FIFO_CONF_REG_ADDR                 0x08U
#define MAX30102_MODE_CONF_REG_ADDR                 0x09U
#define MAX30102_Sp02_CONF_REG_ADDR                 0x0AU
#define MAX30102_LED1_PULSE_AMP_CONF_REG_ADDR       0x0CU
#define MAX30102_LED2_PULSE_AMP_CONF_REG_ADDR       0x0DU
#define MAX30102_MULTILED_MODE_CTRL_REG1_ADDR       0x11U
#define MAX30102_MULTILED_MODE_CTRL_REG2_ADDR       0x12U

//  Die temperature
#define MAX30102_DIE_TEMP_INT_REG_ADDR              0x1FU
#define MAX30102_DIE_TEMP_FRAC_REG_ADDR             0x20U
#define MAX30102_DIE_TEMP_CONFIG_REG_ADDR           0x21U

//  Part ID
#define MAX30102_REV_ID_REG_ADDR                    0xFEU
#define MAX30102_PART_ID_REG_ADDR                   0xFFU

/****************************/
/*      Configurations      */
/****************************/
#define MAX30102_CONF_SAMPLE_AVERAGE_1              0x00U
#define MAX30102_CONF_SAMPLE_AVERAGE_2              0x01U
#define MAX30102_CONF_SAMPLE_AVERAGE_4              0x02U
#define MAX30102_CONF_SAMPLE_AVERAGE_8              0x03U
#define MAX30102_CONF_SAMPLE_AVERAGE_16             0x04U
#define MAX30102_CONF_SAMPLE_AVERAGE_32             0x05U

#define MAX30102_CONF_FIFO_ROLLOVER_EN              0x01U
#define MAX30102_CONF_FIFO_ROLLOVER_DI              0x00U

#define MAX30102_CONF_MODE_HR                       0x02U
#define MAX30102_CONF_MODE_SpO                      0x03U
#define MAX30102_CONF_MODE_MULTI_LED                0x07U

#define MAX30102_CONF_ADC_RANGE_2048                0x00U
#define MAX30102_CONF_ADC_RANGE_4096                0x01U
#define MAX30102_CONF_ADC_RANGE_8192                0x02U
#define MAX30102_CONF_ADC_RANGE_16384               0x03U

#define MAX30102_CONF_SAMPLE_RATE_50                0x00U
#define MAX30102_CONF_SAMPLE_RATE_100               0x01U
#define MAX30102_CONF_SAMPLE_RATE_200               0x02U
#define MAX30102_CONF_SAMPLE_RATE_400               0x03U
#define MAX30102_CONF_SAMPLE_RATE_800               0x04U
#define MAX30102_CONF_SAMPLE_RATE_1000              0x05U
#define MAX30102_CONF_SAMPLE_RATE_1600              0x06U
#define MAX30102_CONF_SAMPLE_RATE_3200              0x07U

#define MAX30102_CONF_LED_PULSE_WIDTH_69            0x00U
#define MAX30102_CONF_LED_PULSE_WIDTH_118           0x01U
#define MAX30102_CONF_LED_PULSE_WIDTH_215           0x02U
#define MAX30102_CONF_LED_PULSE_WIDTH_411           0x03U

//  LED current

#define MAX30102_CONF_LED_SLOT_NONE                 0x00U
#define MAX30102_CONF_LED_SLOT_RED                  0x01U
#define MAX30102_CONF_LED_SLOT_IR                   0x02U


/************************/
/*      Typedefs        */
/************************/

typedef struct {
    uint8_t sampleAverage;
    uint8_t fifoRollover;
    uint8_t fifoAlmostFull;
    uint8_t mode;
    uint8_t adcRange;
    uint8_t sampleRate;
    uint8_t pulseWidth;
    uint8_t pulseAmplitudeRed;
    uint8_t pulseAmplitudeIR;
    uint8_t multiLedSlot1;
    uint8_t multiLedSlot2;
    uint8_t multiLedSlot3;
    uint8_t multiLedSlot4;
} MAX30102_HandleTypeDef;

typedef struct {
    uint8_t x[3];
} MAX30102_DataSample;

typedef struct {
    uint32_t data[MAX30102_SAMPLE_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
} MAX30102_SampleBuffer_td;

/************************/
/*   Global variables   */
/************************/

MAX30102_SampleBuffer_td MAX30102_SampleBuffer;


/************************/
/*      Functions       */
/************************/

//  Setup

uint8_t MAX30102_init( I2C_HandleTypeDef * _pI2C, MAX30102_HandleTypeDef * pInit );

uint8_t MAX30102_reset( void );

uint8_t MAX30102_isReady( void );

uint8_t MAX30102_resetCleared( void );

HAL_StatusTypeDef MAX30102_writeReg( uint8_t memAddr, uint8_t data );

uint8_t MAX30102_readReg( uint8_t memAddr );

uint8_t MAX30102_getPartId( void );

void MAX30102_clearInterrupts( void );

void MAX30102_clearFIFO( void );

void MAX30102_readRegisterValues( void );

//  heart rate stuff

int32_t MAX30102_getHeartRate();

void MAX30102_getData(MAX30102_DataSample * buffer, uint16_t sampleAmount);

int32_t MAX30102_calcHeartRate(uint16_t samplesPerSecond);

uint8_t MAX30102_getAvailableSampleCount();

void MAX30102_smooth(uint16_t lastAddedSampleAmount);

uint32_t MAX30102_sampleToInt(MAX30102_DataSample sample);


//  Sample buffer stuff

void MAX30102_SampleBuffer_add(uint32_t sample);

void MAX30102_SampleBuffer_resetTail();

void MAX30102_SampleBuffer_reverseTail(uint16_t amount);

uint32_t MAX30102_SampleBuffer_readTail();

void MAX30102_SampleBuffer_setTail(uint32_t toSet);

uint32_t MAX30102_SampleBuffer_peek( void );