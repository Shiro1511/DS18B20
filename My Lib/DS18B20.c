#include "DS18B20.h"

extern TIM_HandleTypeDef htim1;

static void delayMicroSeconds(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    HAL_TIM_Base_Start(&htim1);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us)
        ;
    HAL_TIM_Base_Stop(&htim1);
}

static void delayMilliSeconds(uint16_t ms)
{
    for (uint16_t i = 0; i < ms; i++)
    {
        delayMicroSeconds(1000);
    }
}

HAL_StatusTypeDef DS18B20_Reset(DS18B20_HandleTypeDef *ds18b20x)
{
    HAL_StatusTypeDef status;

    /* Set pin as output */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = ds18b20x->GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ds18b20x->GPIOx, &GPIO_InitStruct);

    /* Pull line low for 480us */
    HAL_GPIO_WritePin(ds18b20x->GPIOx, ds18b20x->GPIO_Pin, GPIO_PIN_RESET);
    delayMicroSeconds(480);

    /* Pull line low for 70us */
    HAL_GPIO_WritePin(ds18b20x->GPIOx, ds18b20x->GPIO_Pin, GPIO_PIN_SET);
    delayMicroSeconds(70);

    /* Read the presence pulse */
    /* First, set pin as Input */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(ds18b20x->GPIOx, &GPIO_InitStruct);

    if (HAL_GPIO_ReadPin(ds18b20x->GPIOx, ds18b20x->GPIO_Pin) == GPIO_PIN_SET)
    {
        status = HAL_OK;
    }
    else
    {
        status = HAL_ERROR;
    }

    /* Wait for end of timeslot */
    delayMicroSeconds(410);

    return status;
}

static void DS18B20_WriteBit(DS18B20_HandleTypeDef *ds18b20x, uint8_t bit)
{
    /* Set pin as output */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = ds18b20x->GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ds18b20x->GPIOx, &GPIO_InitStruct);

    /* Pull line low for 1us */
    HAL_GPIO_WritePin(ds18b20x->GPIOx, ds18b20x->GPIO_Pin, GPIO_PIN_RESET);
    delayMicroSeconds(1);

    /* If writing 1, release line */
    if (bit)
    {
        HAL_GPIO_WritePin(ds18b20x->GPIOx, ds18b20x->GPIO_Pin, GPIO_PIN_SET);
    }

    /* Wait for rest of timeslot 60us -> 120us */
    delayMicroSeconds(80);

    /* Release line */
    HAL_GPIO_WritePin(ds18b20x->GPIOx, ds18b20x->GPIO_Pin, GPIO_PIN_SET);
    delayMicroSeconds(2);
}

static uint8_t DS18B20_ReadBit(DS18B20_HandleTypeDef *ds18b20x)
{
    uint8_t bit_read = 0;

    /* Set pin as Output Open-Drain */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = ds18b20x->GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ds18b20x->GPIOx, &GPIO_InitStruct);

    /* Pull line low for 2us */
    HAL_GPIO_WritePin(ds18b20x->GPIOx, ds18b20x->GPIO_Pin, GPIO_PIN_RESET);
    delayMicroSeconds(2);

    /* Release line and switch to Input */
    HAL_GPIO_WritePin(ds18b20x->GPIOx, ds18b20x->GPIO_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ds18b20x->GPIOx, &GPIO_InitStruct);

    /* Wait 10-15us then read */
    delayMicroSeconds(15);

    /* Read the line state */
    if (HAL_GPIO_ReadPin(ds18b20x->GPIOx, ds18b20x->GPIO_Pin))
    {
        bit_read = 1;
    }

    /* Wait for rest of timeslot */
    delayMicroSeconds(45);

    return bit_read;
}

void DS18B20_WriteByte(DS18B20_HandleTypeDef *ds18b20x, uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        DS18B20_WriteBit(ds18b20x, data & 0x01);
        data = data >> 1; /* data >>= 1 */
    }
}

uint8_t DS18B20_ReadByte(DS18B20_HandleTypeDef *ds18b20x)
{
    uint8_t data = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        data = data >> 1; /* data >>= 1 */
        if (DS18B20_ReadBit(ds18b20x))
        {
            data |= 0x80;
        }
    }

    return data;
}

DS18B20_Status_t DS18B20_StartConversion(DS18B20_HandleTypeDef *ds18b20x)
{
    if (DS18B20_Reset(ds18b20x) != HAL_OK)
    {
        return DS18B20_NO_SENSOR;
    }

    DS18B20_WriteByte(ds18b20x, DS18B20_CMD_SKIP_ROM);
    DS18B20_WriteByte(ds18b20x, DS18B20_CMD_CONVERT_TEMP);

    switch (ds18b20x->Resolution)
    {
    case DS18B20_RESOLUTION_9BIT:
        delayMilliSeconds(94);
        break;
    case DS18B20_RESOLUTION_10BIT:
        delayMilliSeconds(188);
        break;
    case DS18B20_RESOLUTION_11BIT:
        delayMilliSeconds(375);
        break;
    case DS18B20_RESOLUTION_12BIT:
    default:
        delayMilliSeconds(750);
        break;
    }

    return DS18B20_OK;
}

static uint8_t DS18B20_CRC_Update(uint8_t crc, uint8_t data)
{
    uint8_t i;

    crc = crc ^ data;
    for (i = 0; i < 8; i++)
    {
        if (crc & 0x01)
        {
            crc = (crc >> 1) ^ 0x8C;
        }
        else
        {
            crc >>= 1;
        }
    }

    return crc;
}

static DS18B20_Status_t DS18B20_CRC_Check(uint8_t *data, uint8_t length, uint8_t crc)
{
    uint8_t crc_calc = 0;

    for (uint8_t i = 0; i < length; i++)
    {
        crc_calc = DS18B20_CRC_Update(crc_calc, data[i]);
    }

    return (crc_calc == crc) ? DS18B20_OK : DS18B20_CRC_ERROR;
}

DS18B20_Status_t DS18B20_ReadTemperature(DS18B20_HandleTypeDef *ds18b20x, float *temperature)
{
    uint8_t data[9];
    uint16_t temp_raw;

    /* Check if sensor is present */
    if (DS18B20_Reset(ds18b20x) != HAL_OK)
    {
        return DS18B20_NO_SENSOR;
    }

    /* Read scratchpad */
    DS18B20_WriteByte(ds18b20x, DS18B20_CMD_SKIP_ROM);
    DS18B20_WriteByte(ds18b20x, DS18B20_CMD_READ_SCRATCHPAD);

    /* Read 9 bytes from scratchpad */
    for (uint8_t i = 0; i < 9; i++)
    {
        data[i] = DS18B20_ReadByte(ds18b20x);
    }

    /* Verify CRC */
    if (DS18B20_CRC_Check(data, 8, data[8]) != DS18B20_OK)
    {
        return DS18B20_CRC_ERROR;
    }

    /* Convert raw data to temperature */
    temp_raw = (data[1] << 8) | data[0];

    /* Handle negative temperatures */
    if (temp_raw & 0x8000)
    {
        temp_raw = ~temp_raw + 1;
        *temperature = -(float)temp_raw / 16.0f;
    }
    else
    {
        *temperature = (float)temp_raw / 16.0f;
    }

    ds18b20x->LastTemperature = *temperature;

    return DS18B20_OK;
}

DS18B20_Status_t DS18B20_CheckSensor(DS18B20_HandleTypeDef *ds18b20x)
{
    return (DS18B20_Reset(ds18b20x) == HAL_OK) ? DS18B20_OK : DS18B20_NO_SENSOR;
}

DS18B20_Status_t DS18B20_ReadROM(DS18B20_HandleTypeDef *ds18b20x, uint8_t *rom)
{
    if (DS18B20_Reset(ds18b20x) != HAL_OK)
    {
        return DS18B20_NO_SENSOR;
    }

    DS18B20_WriteByte(ds18b20x, DS18B20_CMD_READ_ROM);

    for (uint8_t i = 0; i < 8; i++)
    {
        rom[i] = DS18B20_ReadByte(ds18b20x);
    }

    return DS18B20_OK;
}

DS18B20_Status_t DS18B20_SetResolution(DS18B20_HandleTypeDef *ds18b20x, DS18B20_Resolution_t resolution)
{
    if (DS18B20_Reset(ds18b20x) != HAL_OK)
    {
        return DS18B20_NO_SENSOR;
    }

    DS18B20_WriteByte(ds18b20x, DS18B20_CMD_SKIP_ROM);
    DS18B20_WriteByte(ds18b20x, DS18B20_CMD_WRITE_SCRATCHPAD);
    DS18B20_WriteByte(ds18b20x, 0x00);       // TH register
    DS18B20_WriteByte(ds18b20x, 0x00);       // TL register
    DS18B20_WriteByte(ds18b20x, resolution); // Configuration register

    ds18b20x->Resolution = resolution;

    return DS18B20_OK;
}

void DS18B20_Init(DS18B20_HandleTypeDef *ds18b20x, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    ds18b20x->GPIOx = GPIOx;
    ds18b20x->GPIO_Pin = GPIO_Pin;
    ds18b20x->Resolution = DS18B20_RESOLUTION_12BIT;
    ds18b20x->LastTemperature = -999.0f;

    /* Configure GPIO as Open Drain Output */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = ds18b20x->GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ds18b20x->GPIOx, &GPIO_InitStruct);

    /* Set pin high initially */
    HAL_GPIO_WritePin(ds18b20x->GPIOx, ds18b20x->GPIO_Pin, GPIO_PIN_SET);
}