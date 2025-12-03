#ifndef _DS18B20_H_
#define _DS18B20_H_

#include "stm32f1xx_hal.h"

/**
 * @brief  DS18B20 command set used for ROM operations,
 *         temperature conversion, and scratchpad access.
 */
#define DS18B20_CMD_SEARCH_ROM 0xF0
#define DS18B20_CMD_READ_ROM 0x33
#define DS18B20_CMD_MATCH_ROM 0x55
#define DS18B20_CMD_SKIP_ROM 0xCC
#define DS18B20_CMD_ALARM_SEARCH 0xEC
#define DS18B20_CMD_CONVERT_TEMP 0x44
#define DS18B20_CMD_WRITE_SCRATCHPAD 0x4E
#define DS18B20_CMD_READ_SCRATCHPAD 0xBE
#define DS18B20_CMD_COPY_SCRATCHPAD 0x48
#define DS18B20_CMD_RECALL_EEPROM 0xB8
#define DS18B20_CMD_READ_POWER_SUPPLY 0xB4

/**
 * @brief  DS18B20 temperature resolution configuration.
 *         Conversion time increases with resolution.
 */
typedef enum
{
    DS18B20_RESOLUTION_9BIT = 0x1F,  /*!<  9-bit resolution, 93.75ms conversion */
    DS18B20_RESOLUTION_10BIT = 0x3F, /*!< 10-bit resolution, 187.5ms conversion */
    DS18B20_RESOLUTION_11BIT = 0x5F, /*!< 11-bit resolution, 375ms conversion  */
    DS18B20_RESOLUTION_12BIT = 0x7F  /*!< 12-bit resolution, 750ms conversion  */
} DS18B20_Resolution_t;

/**
 * @brief  DS18B20 driver status codes.
 */
typedef enum
{
    DS18B20_OK = 0x00,           /*!< Operation successful */
    DS18B20_ERROR = 0x01,        /*!< Generic error */
    DS18B20_NO_SENSOR = 0x02,    /*!< Sensor not detected */
    DS18B20_CRC_ERROR = 0x03,    /*!< Scratchpad CRC mismatch */
    DS18B20_TIMEOUT_ERROR = 0x04 /*!< Timing timeout occurred */
} DS18B20_Status_t;

/**
 * @brief  DS18B20 device handle structure.
 * @note   ROM_Address[0] contains family code (0x28 for DS18B20).
 */
typedef struct
{
    GPIO_TypeDef *GPIOx;             /*!< GPIO port used for 1-Wire line */
    uint16_t GPIO_Pin;               /*!< GPIO pin used for 1-Wire line */
    uint8_t ROM_Address[8];          /*!< Unique 64-bit ROM code */
    DS18B20_Resolution_t Resolution; /*!< Current resolution setting */
    float LastTemperature;           /*!< Last measured temperature (°C) */
} DS18B20_HandleTypeDef;

/**
 * @brief  Attach a hardware timer for microsecond delays.
 * @note   This timer will be used by the DS18B20 driver for precise timing.
 * @param  htim: Pointer to a TIM_HandleTypeDef structure
 * @retval None
 */
void DS18B20_SetTimerHandle(TIM_HandleTypeDef *htim);

/**
 * @brief  Initialize DS18B20 sensor and prepare 1-Wire communication.
 * @param  ds18b20x: Pointer to DS18B20 handle structure
 * @param  GPIOx: GPIO Port used for data line
 * @param  GPIO_Pin: GPIO Pin used for data line
 * @retval None
 */
void DS18B20_Init(DS18B20_HandleTypeDef *ds18b20x,
                  GPIO_TypeDef *GPIOx,
                  uint16_t GPIO_Pin);

/**
 * @brief  Start temperature conversion on DS18B20.
 * @param  ds18b20x: Pointer to DS18B20 handle structure
 * @retval DS18B20_Status_t: Operation result
 */
DS18B20_Status_t DS18B20_StartConversion(DS18B20_HandleTypeDef *ds18b20x);

/**
 * @brief  Read temperature value from DS18B20 scratchpad.
 * @param  ds18b20x: Pointer to DS18B20 handle structure
 * @param  temperature: Pointer to float storing the temperature
 * @retval DS18B20_Status_t: Operation result
 */
DS18B20_Status_t DS18B20_ReadTemperature(DS18B20_HandleTypeDef *ds18b20x,
                                         float *temperature);

/**
 * @brief  Configure DS18B20 resolution.
 * @param  ds18b20x: Pointer to DS18B20 handle structure
 * @param  resolution: Desired resolution setting
 * @retval DS18B20_Status_t: Operation result
 */
DS18B20_Status_t DS18B20_SetResolution(DS18B20_HandleTypeDef *ds18b20x,
                                       DS18B20_Resolution_t resolution);

/**
 * @brief  Read unique 64-bit ROM code (only valid when one sensor is on bus).
 * @param  ds18b20x: Pointer to DS18B20 handle structure
 * @param  rom: Pointer to 8-byte buffer to store ROM code
 * @retval DS18B20_Status_t: Operation result
 */
DS18B20_Status_t DS18B20_ReadROM(DS18B20_HandleTypeDef *ds18b20x,
                                 uint8_t *rom);

/**
 * @brief  Check if DS18B20 is responding on the 1-Wire bus.
 * @param  ds18b20x: Pointer to DS18B20 handle structure
 * @retval DS18B20_Status_t: DS18B20_OK if sensor detected
 */
DS18B20_Status_t DS18B20_CheckSensor(DS18B20_HandleTypeDef *ds18b20x);

#endif /* _DS18B20_H_ */