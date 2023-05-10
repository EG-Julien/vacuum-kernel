#ifndef VACUUMIZER_H
#define VACUUMIZER_H

#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

#define UART_TX_PIN 0
#define UART_RX_PIN 1

#include <stdarg.h>
#include "adxl345_driver.h"
#include "adxl345_driver_interface.h"

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @defgroup adxl345_example_driver adxl345 example driver function
 * @brief    adxl345 example driver modules
 * @ingroup  adxl345_driver
 * @{
 */

/**
 * @brief adxl345 basic example default definition
 */
#define ADXL345_BASIC_DEFAULT_RATE                        ADXL345_RATE_100                      /**< rate 100Hz */
#define ADXL345_BASIC_DEFAULT_SPI_WIRE                    ADXL345_SPI_WIRE_4                    /**< spi wire 4 */
#define ADXL345_BASIC_DEFAULT_INTERRUPT_ACTIVE_LEVEL      ADXL345_INTERRUPT_ACTIVE_LEVEL_LOW    /**< interrupt pin low */
#define ADXL345_BASIC_DEFAULT_FULL_RESOLUTION             ADXL345_BOOL_TRUE                     /**< enable full resolution */
#define ADXL345_BASIC_DEFAULT_AUTO_SLEEP                  ADXL345_BOOL_FALSE                    /**< disable auto sleep */
#define ADXL345_BASIC_DEFAULT_SLEEP                       ADXL345_BOOL_FALSE                    /**< disable sleep */
#define ADXL345_BASIC_DEFAULT_SLEEP_FREQUENCY             ADXL345_SLEEP_FREQUENCY_1HZ           /**< sleep frequency 1Hz */
#define ADXL345_BASIC_DEFAULT_JUSTIFY                     ADXL345_JUSTIFY_RIGHT                 /**< justify right */
#define ADXL345_BASIC_DEFAULT_RANGE                       ADXL345_RANGE_2G                      /**< range 2g */
#define ADXL345_BASIC_DEFAULT_MODE                        ADXL345_MODE_BYPASS                   /**< bypass mode */
#define ADXL345_BASIC_DEFAULT_TRIGGER_PIN                 ADXL345_INTERRUPT_PIN2                /**< trigger pin map interrupt pin 2 */
#define ADXL345_BASIC_DEFAULT_TAP_SUPPRESS                ADXL345_BOOL_FALSE                    /**< disable tap suppress */
#define ADXL345_BASIC_DEFAULT_INTERRUPT_SINGLE_TAP_MAP    ADXL345_INTERRUPT_PIN1                /**< single tap map interrupt pin 1 */
#define ADXL345_BASIC_DEFAULT_INTERRUPT_DOUBLE_TAP_MAP    ADXL345_INTERRUPT_PIN1                /**< double tap map interrupt pin 1 */
#define ADXL345_BASIC_DEFAULT_INTERRUPT_ACTIVITY_MAP      ADXL345_INTERRUPT_PIN1                /**< activity map interrupt pin 1 */
#define ADXL345_BASIC_DEFAULT_INTERRUPT_INACTIVITY_MAP    ADXL345_INTERRUPT_PIN1                /**< inactivity map interrupt pin 1 */
#define ADXL345_BASIC_DEFAULT_INTERRUPT_FREE_FALL_MAP     ADXL345_INTERRUPT_PIN1                /**< free fall map interrupt pin 1 */
#define ADXL345_BASIC_DEFAULT_INTERRUPT_DATA_READY_MAP    ADXL345_INTERRUPT_PIN1                /**< data ready map interrupt pin 1 */
#define ADXL345_BASIC_DEFAULT_INTERRUPT_WATERMARK_MAP     ADXL345_INTERRUPT_PIN1                /**< watermark map interrupt pin 1 */
#define ADXL345_BASIC_DEFAULT_INTERRUPT_OVERRUN_MAP       ADXL345_INTERRUPT_PIN1                /**< overrun map interrupt pin 1 */
#define ADXL345_BASIC_DEFAULT_LINK_ACTIVITY_INACTIVITY    ADXL345_BOOL_TRUE                     /**< enable activity inactivity */
#define ADXL345_BASIC_DEFAULT_INTERRUPT_DATA_READY        ADXL345_BOOL_FALSE                    /**< disable data ready */
#define ADXL345_BASIC_DEFAULT_INTERRUPT_WATERMARK         ADXL345_BOOL_FALSE                    /**< disable watermark */
#define ADXL345_BASIC_DEFAULT_INTERRUPT_OVERRUN           ADXL345_BOOL_FALSE                    /**< disable overrun */
#define ADXL345_BASIC_DEFAULT_ACTION_COUPLED              ADXL345_COUPLED_AC                    /**< action ac coupled */
#define ADXL345_BASIC_DEFAULT_INACTION_COUPLED            ADXL345_COUPLED_DC                    /**< inaction dc coupled */
#define ADXL345_BASIC_DEFAULT_WATERMARK                   16                                    /**< watermark 16 level */
#define ADXL345_BASIC_DEFAULT_OFFSET                      0.0f                                  /**< 0 offset */
#define ADXL345_BASIC_DEFAULT_TAP_THRESHOLD               3.0f                                  /**< tap threshold 3.0g */
#define ADXL345_BASIC_DEFAULT_DURATION                    10 * 1000                             /**< duration 10 ms */
#define ADXL345_BASIC_DEFAULT_LATENT                      20.0f                                 /**< latent 20 ms */
#define ADXL345_BASIC_DEFAULT_WINDOW                      80.0f                                 /**< window 80 ms */
#define ADXL345_BASIC_DEFAULT_ACTION_THRESHOLD            2.0f                                  /**< action threshold 2g */
#define ADXL345_BASIC_DEFAULT_INACTION_THRESHOLD          1.0f                                  /**< inaction threshold 1g */
#define ADXL345_BASIC_DEFAULT_INACTION_TIME               3                                     /**< inaction 3s */
#define ADXL345_BASIC_DEFAULT_FREE_FALL_THRESHOLD         0.8f                                  /**< free fall threshold 0.8g */
#define ADXL345_BASIC_DEFAULT_FREE_FALL_TIME              10                                    /**< free fall time 10 ms */

/**
 * @brief     basic example init
 * @param[in] interface is the chip interface
 * @param[in] addr_pin is the iic device address
 * @return    status code
 *            - 0 success
 *            - 1 init failed
 * @note      none
 */
uint8_t adxl345_basic_init(adxl345_interface_t interface, adxl345_address_t addr_pin);

/**
 * @brief  basic example deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t adxl345_basic_deinit(void);

/**
 * @brief      basic example read
 * @param[out] *g points to a converted data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t adxl345_basic_read(float g[3]);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

void uart_print(const char *fmt, ...);

#endif