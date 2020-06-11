#include "MAX30102.h"

I2C_HandleTypeDef * pI2C;

// struct testStruct {
//     uint8_t sampleCount;
//     uint8_t readPtr;
//     uint8_t writePtr;
//     uint8_t ofvCounter;
// };

// struct testStruct testBuffer[50] = {0};
// uint32_t testBufferCounter = 0;

//  Setup

/* Initialize the MAX30102 peripheral */
uint8_t MAX30102_init( I2C_HandleTypeDef * _pI2C, MAX30102_HandleTypeDef * pInit ) {
    uint8_t data = 0;
    uint8_t partID __attribute__((used)) = 0;
    
    pI2C = _pI2C;

    //  Reset to power on state
    if (MAX30102_reset() != HAL_OK) {
        return 1;
    }

    partID = MAX30102_getPartId();
    if (partID != 0x15) {
        return 1;
    }



    //  Interrupt enable/disable
    data = 0x0U;
    if (MAX30102_writeReg(MAX30102_IT_ENABLE1_REG_ADDR, data) != HAL_OK) {
        return 1;
    } 

    data = 0x0U;
    if (MAX30102_writeReg(MAX30102_IT_ENABLE2_REG_ADDR, data) != HAL_OK) {
        return 1;
    }


    //  FIFO/ADC setup
    data = (pInit->sampleAverage << 5) | (pInit->fifoRollover << 4) | pInit->fifoAlmostFull;   
    if (MAX30102_writeReg(MAX30102_FIFO_CONF_REG_ADDR, data) != HAL_OK) {
        return 1;
    }

    data = (pInit->adcRange << 5) | (pInit->sampleRate << 2) | pInit->pulseWidth;             
    if (MAX30102_writeReg(MAX30102_Sp02_CONF_REG_ADDR, data) != HAL_OK) {
        return 1;
    }

    data = pInit->mode;                                                                       
    if (MAX30102_writeReg(MAX30102_MODE_CONF_REG_ADDR, data) != HAL_OK) {
        return 1;
    }


    //  LED setup
    data = (pInit->multiLedSlot2 << 4) | pInit->multiLedSlot1;                                
    if (MAX30102_writeReg(MAX30102_MULTILED_MODE_CTRL_REG1_ADDR, data) != HAL_OK) {
        return 1;
    }

    data = (pInit->multiLedSlot4 << 4) | pInit->multiLedSlot3;
    if (MAX30102_writeReg(MAX30102_MULTILED_MODE_CTRL_REG2_ADDR, data) != HAL_OK) {
        return 1;
    }

    data = pInit->pulseAmplitudeRed;                                                        
    if (MAX30102_writeReg(MAX30102_LED1_PULSE_AMP_CONF_REG_ADDR, data) != HAL_OK) {
        return 1;
    }

    data = pInit->pulseAmplitudeIR;                                                            
    if (MAX30102_writeReg(MAX30102_LED2_PULSE_AMP_CONF_REG_ADDR, data) != HAL_OK) {
        return 1;
    }

    MAX30102_clearFIFO();

    return 0;
}

//  Resets the sensor to power on state
uint8_t MAX30102_reset( void ) {

	if (MAX30102_writeReg(MAX30102_MODE_CONF_REG_ADDR, 0x40U) != HAL_OK) {
        return 1;
    }
    
    while (!MAX30102_resetCleared()) {
        asm("nop");
    }

    MAX30102_clearInterrupts();

    return 0;
}

//  Check if reset has been complete
uint8_t MAX30102_resetCleared( void ) {
    uint8_t regData = MAX30102_readReg(MAX30102_MODE_CONF_REG_ADDR);
    if (regData & 0x40) {
        return 0;
    } else {
        return 1;
    }
}

/*  Check if device has powered on 
 *  The power ready flag is creared once the register is read
 */
uint8_t MAX30102_isReady( void ) {
    uint8_t regData = MAX30102_readReg(MAX30102_IT_STATUS1_REG_ADDR);
    if (regData & 1) {
        return 1;
    } else {
        return 0;
    }
}

void MAX30102_clearInterrupts( void ) {
    MAX30102_readReg(MAX30102_IT_STATUS1_REG_ADDR);
    MAX30102_readReg(MAX30102_IT_STATUS2_REG_ADDR);
}

//  Part ID should always return 0x15
uint8_t MAX30102_getPartId( void ) {
    return MAX30102_readReg(MAX30102_PART_ID_REG_ADDR);
}

void MAX30102_clearFIFO( void ) {
    MAX30102_writeReg(MAX30102_FIFO_WR_PTR_REG_ADDR, 0x0U);
    MAX30102_writeReg(MAX30102_OFV_COUNTER_REG_ADDR, 0x0U);
    MAX30102_writeReg(MAX30102_FIFO_RD_PTR_REG_ADDR, 0x0U);
}

void MAX30102_readRegisterValues( void ) {
    uint8_t values[0x13] __attribute__((used));
    for (int i = 0; i <= 0x12; i++) {
        values[i] = MAX30102_readReg(i);
    }
    asm("NOP");
}

//  Heart rate stuff
int32_t MAX30102_getHeartRate() {

    uint16_t sampleRate = 25;

    uint8_t availableSampleCount;
    MAX30102_DataSample samples[32];
    uint32_t translatedSample;

    //  get how many new samples available
    availableSampleCount = MAX30102_getAvailableSampleCount();

    // testBuffer[testBufferCounter].sampleCount = availableSampleCount;

    //  allocate memory for samples
    // samples = (MAX30102_DataSample *) malloc(sizeof(MAX30102_DataSample) * availableSampleCount);

    //  get available samples
    MAX30102_getData(samples, availableSampleCount);

    //  add new samples to sample storage
    for (int i = 0; i < availableSampleCount; i++) {
        translatedSample = MAX30102_sampleToInt(samples[i]);
        MAX30102_SampleBuffer_add(translatedSample);
    }

    //  Smooth the new data
    MAX30102_smooth(availableSampleCount);

    // free(samples);
    // testBufferCounter++;

    //  calculate heart rate
    return MAX30102_calcHeartRate(sampleRate);
}



void MAX30102_smooth(uint16_t lastAddedSampleCount) {
    uint8_t smoothingNumber = 5;
    
    MAX30102_SampleBuffer_resetTail();
    MAX30102_SampleBuffer_reverseTail(lastAddedSampleCount + smoothingNumber);

    for (int i = 0; i < smoothingNumber; i++) {
        
    }

}

int32_t MAX30102_calcHeartRate(uint16_t samplesPerSecond) {

    int amountOfSamples = MAX30102_SAMPLE_BUFFER_SIZE;

    float beatsPerSecond;
    float beatCounter = 0;

    uint32_t lastSample;
    uint32_t currentSample;
    _Bool ascending = 1;
    uint8_t transitionCounter = 0;

    MAX30102_SampleBuffer_resetTail();
    currentSample = MAX30102_SampleBuffer_readTail();
    lastSample = currentSample;

    for (int i = 0; i < MAX30102_SAMPLE_BUFFER_SIZE; i++) {
        currentSample = MAX30102_SampleBuffer_readTail();

        if (ascending) {
            if ( currentSample < lastSample) {
                transitionCounter++; 
                if (transitionCounter >= 3) {
                    beatCounter++;
                    ascending = 0;
                }
            } else {
                transitionCounter = 0;
            }
        } else {
            if (currentSample > lastSample) {
                transitionCounter++;
                if (transitionCounter >= 3) {
                    ascending = 1;
                }
            } else {
                transitionCounter = 0;
            }
        }
        lastSample = currentSample;
    }

    beatsPerSecond = beatCounter / ((float) (amountOfSamples / samplesPerSecond));
    return (int32_t) (beatsPerSecond * 60);
}

uint32_t MAX30102_sampleToInt(MAX30102_DataSample sample) {
    uint32_t ret = 0 | (uint32_t) sample.x[2];
    ret |= (((uint32_t) sample.x[1]) << 8);
    ret |= (((uint32_t) sample.x[0]) << 16);

    return ret;
}

uint8_t MAX30102_getAvailableSampleCount() {
    uint8_t readPtr = MAX30102_readReg(MAX30102_FIFO_RD_PTR_REG_ADDR);
    uint8_t writePtr = MAX30102_readReg(MAX30102_FIFO_WR_PTR_REG_ADDR);

    // testBuffer[testBufferCounter].readPtr = readPtr;
    // testBuffer[testBufferCounter].writePtr = writePtr;
    // testBuffer[testBufferCounter].ofvCounter = MAX30102_readReg(MAX30102_OFV_COUNTER_REG_ADDR);

    return writePtr - readPtr;
}

void MAX30102_getData(MAX30102_DataSample * buffer, uint16_t sampleAmount) {
    uint16_t size = sampleAmount * 3;
    HAL_I2C_Mem_Read(pI2C, MAX30102_READ_ADDR, MAX30102_FIFO_DATA_REG_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t *) buffer, size, 10);
    MAX30102_writeReg(MAX30102_FIFO_WR_PTR_REG_ADDR, 0x0U);
    MAX30102_writeReg(MAX30102_FIFO_RD_PTR_REG_ADDR, 0x0U);
}





//  Read/Write helpers
uint8_t MAX30102_readReg( uint8_t memAddr ) {
    uint8_t data = 0;
    HAL_StatusTypeDef status __attribute__((used)) =  HAL_I2C_Mem_Read(pI2C, MAX30102_READ_ADDR, memAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
    return data;
}

HAL_StatusTypeDef MAX30102_writeReg( uint8_t memAddr, uint8_t data ) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(pI2C, MAX30102_WRITE_ADDR, memAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
    return status;
}


//  Sample buffer code
void MAX30102_SampleBuffer_add(uint32_t sample) {
    MAX30102_SampleBuffer.data[MAX30102_SampleBuffer.head] = sample;

    MAX30102_SampleBuffer.head++;

    if (MAX30102_SampleBuffer.head == MAX30102_SAMPLE_BUFFER_SIZE) {
        MAX30102_SampleBuffer.head = 0;
    }
}

void MAX30102_SampleBuffer_resetTail() {
    MAX30102_SampleBuffer.tail = MAX30102_SampleBuffer.head + 1;

    if (MAX30102_SampleBuffer.tail == MAX30102_SAMPLE_BUFFER_SIZE) {
        MAX30102_SampleBuffer.tail = 0;
    }
}

uint32_t MAX30102_SampleBuffer_readTail() {
    uint32_t ret = MAX30102_SampleBuffer.data[MAX30102_SampleBuffer.tail];

    MAX30102_SampleBuffer.tail++;
    
    if (MAX30102_SampleBuffer.tail == MAX30102_SAMPLE_BUFFER_SIZE) {
        MAX30102_SampleBuffer.tail = 0;
    }

    return ret;
}

void MAX30102_SampleBuffer_setTail(uint32_t toSet) {
    MAX30102_SampleBuffer.data[MAX30102_SampleBuffer.tail] = toSet;
}

/*
*   set the tail back x places
*/
void MAX30102_SampleBuffer_reverseTail(uint16_t amount) {
    if (MAX30102_SampleBuffer.tail < amount) {
        amount -= MAX30102_SampleBuffer.tail;
        MAX30102_SampleBuffer.tail = MAX30102_SAMPLE_BUFFER_SIZE - amount;
    } else {
        MAX30102_SampleBuffer.tail -= amount;
    }
}

uint32_t MAX30102_SampleBuffer_peek( void ) {
    return MAX30102_SampleBuffer.data[MAX30102_SampleBuffer.tail];
}