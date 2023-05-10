/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_adxl345_interface_template.c
 * @brief     driver adxl345 interface template source file
 * @version   2.0.0
 * @author    Shifeng Li
 * @date      2021-04-20
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2021/04/20  <td>2.0      <td>Shifeng Li  <td>format the code
 * <tr><td>2020/12/23  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "adxl345_driver_interface.h"

/**
 * Macro used to cs
*/

static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t adxl345_interface_iic_init(void)
{
    i2c_init(i2c_default, 100 * 1000); // defaulting i2c interface to i2c_default
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t adxl345_interface_iic_deinit(void)
{
    return 0;
}

/**
 * @brief      interface iic bus read
 * @param[in]  addr is the iic device write address
 * @param[in]  reg is the iic register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t adxl345_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t err = 0;

    err += i2c_write_blocking(i2c_default, addr, &reg, 1, true);
    err += i2c_read_blocking(i2c_default, addr, buf, len, false);

    if (err > 0)
        return 0;

    return 1;
}

/**
 * @brief     interface iic bus write
 * @param[in] addr is the iic device write address
 * @param[in] reg is the iic register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t adxl345_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t buffer[len + 1];

    for (uint8_t i = 0; i < (len + 1); i++)
    {
        if (i == 0) {
            buffer[i] = reg;
        } else {
            buffer[i] = buf[i - 1];
        }
    }

    uint8_t err = i2c_write_blocking(i2c_default, addr, buffer, (len + 1), false);
    if (err > 0)
        return 0;
    return 1;
}

/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t adxl345_interface_spi_init(void)
{
    spi_init(spi_default, 500 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(PIN_MISO, PIN_MOSI, PIN_SCK, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(PIN_CS, "SPI CS"));

    return 0;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t adxl345_interface_spi_deinit(void)
{   
    return 0;
}

/**
 * @brief      interface spi bus read
 * @param[in]  reg is the register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t adxl345_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    reg |= READ_BIT;
    cs_select();
    spi_write_blocking(spi_default, &reg, 1);
    sleep_ms(10);
    spi_read_blocking(spi_default, 0, buf, len);
    cs_deselect();
    sleep_ms(10);
    return 0;
}

/**
 * @brief     interface spi bus write
 * @param[in] reg is the register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t adxl345_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t buffer[len + 1];

    for (uint8_t i = 0; i < (len + 1); i++)
    {
        if (i == 0) {
            buffer[i] = reg;
        } else {
            buffer[i] = buf[i - 1];
        }
    }

    cs_select();
    spi_write_blocking(spi_default, buffer, (len + 1));
    return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms
 * @note      none
 */
void adxl345_interface_delay_ms(uint32_t ms)
{
    sleep_ms(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt is the format data
 * @note      none
 */
void adxl345_interface_debug_print(const char *const fmt, ...)
{
    va_list ap;
    int ret;
    char buffer[64];

    va_start(ap, fmt);
    ret = vsprintf(buffer, fmt, ap);
    va_end(ap);
    
    for (uint8_t i = 0; i < 64; i++)
    {
        if (buffer[i] == 0)
            return;
        uart_putc(uart0, buffer[i]);
    }
}

/**
 * @brief     interface receive callback
 * @param[in] type is the irq type
 * @note      none
 */
void adxl345_interface_receive_callback(uint8_t type)
{
    switch (type)
    {
        case ADXL345_INTERRUPT_DATA_READY :
        {
            adxl345_interface_debug_print("adxl345: irq data ready.\n");
            
            break;
        }
        case ADXL345_INTERRUPT_SINGLE_TAP :
        {
            adxl345_interface_debug_print("adxl345: irq single tap.\n");
            
            break;
        }
        case ADXL345_INTERRUPT_DOUBLE_TAP :
        {
            adxl345_interface_debug_print("adxl345: irq double tap.\n");
            
            break;
        }
        case ADXL345_INTERRUPT_ACTIVITY :
        {
            adxl345_interface_debug_print("adxl345: irq activity.\n");
            
            break;
        }
        case ADXL345_INTERRUPT_INACTIVITY :
        {
            adxl345_interface_debug_print("adxl345: irq inactivity.\n");
            
            break;
        }
        case ADXL345_INTERRUPT_FREE_FALL :
        {
            adxl345_interface_debug_print("adxl345: irq free fall.\n");
            
            break;
        }
        case ADXL345_INTERRUPT_WATERMARK :
        {
            adxl345_interface_debug_print("adxl345: irq water mark.\n");
            
            break;
        }
        case ADXL345_INTERRUPT_OVERRUN :
        {
            adxl345_interface_debug_print("adxl345: irq overrun.\n");
            
            break;
        }
        default :
        {
            adxl345_interface_debug_print("adxl345: unknown code.\n");
            
            break;
        }
    }
}