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
 * @file      driver_w25qxx_interface_template.c
 * @brief     driver w25qxx interface template source file
 * @version   1.0.0
 * @date      2021-07-15
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2021/07/15  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_w25qxx_interface.h"
#include "main.h"
#include "nrf_delay.h"
#include "nrfx_spim.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include <stdarg.h>
#include <stdio.h>

/**
 * @brief  interface spi qspi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi qspi init failed
 * @note   none
 */

#define W25QXX_SPI_INSTANCE 0 // SPIM instance to use
static const nrfx_spim_t spim_instance = NRFX_SPIM_INSTANCE(W25QXX_SPI_INSTANCE);
static volatile bool spi_xfer_done;

static uint8_t tx_buf[4096 + 8] = {0xff}; // Adjust size as needed
static uint8_t rx_buf[4096 + 8] = {0xff};

void spi_event_handler(nrfx_spim_evt_t const *p_event, void *p_context) {
  spi_xfer_done = true;
}

uint8_t w25qxx_interface_spi_qspi_init(void) {
  nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG;
  spim_config.ss_pin = NRFX_SPIM_PIN_NOT_USED; // We'll handle CS manually
  spim_config.miso_pin = W25QXX_SPI_MISO_PIN;
  spim_config.mosi_pin = W25QXX_SPI_MOSI_PIN;
  spim_config.sck_pin = W25QXX_SPI_SCK_PIN;
  spim_config.frequency = NRF_SPIM_FREQ_8M; // Set frequency as needed
  spim_config.mode = NRF_SPIM_MODE_0;       // SPI mode 0 (CPOL = 0, CPHA = 0)
  spim_config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;

  ret_code_t ret = nrfx_spim_init(&spim_instance, &spim_config, spi_event_handler, NULL);
  if (ret != NRF_SUCCESS) {
    return 1; // Initialization failed
  }

  // Configure CS pin as output and set it high (inactive)
  nrf_gpio_cfg_output(W25QXX_SPI_CS_PIN);
  nrf_gpio_pin_set(W25QXX_SPI_CS_PIN);

  return 0; // Success
}

/**
 * @brief  interface spi qspi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi qspi deinit failed
 * @note   none
 */
uint8_t w25qxx_interface_spi_qspi_deinit(void) {
  nrfx_spim_uninit(&spim_instance);
  return 0; // Success
}

/**
 * @brief      interface spi qspi bus write read
 * @param[in]  instruction is the sent instruction
 * @param[in]  instruction_line is the instruction phy lines
 * @param[in]  address is the register address
 * @param[in]  address_line is the address phy lines
 * @param[in]  address_len is the address length
 * @param[in]  alternate is the register address
 * @param[in]  alternate_line is the alternate phy lines
 * @param[in]  alternate_len is the alternate length
 * @param[in]  dummy is the dummy cycle
 * @param[in]  *in_buf points to a input buffer
 * @param[in]  in_len is the input length
 * @param[out] *out_buf points to a output buffer
 * @param[in]  out_len is the output length
 * @param[in]  data_line is the data phy lines
 * @return     status code
 *             - 0 success
 *             - 1 write read failed
 * @note       none
 */
uint8_t w25qxx_interface_spi_qspi_write_read(uint8_t instruction, uint8_t instruction_line,
    uint32_t address, uint8_t address_line, uint8_t address_len,
    uint32_t alternate, uint8_t alternate_line, uint8_t alternate_len,
    uint8_t dummy, uint8_t *in_buf, uint32_t in_len,
    uint8_t *out_buf, uint32_t out_len, uint8_t data_line) {

  uint32_t tx_index = 0;

  // Build tx_buf
  if (instruction != 0x00) {
    tx_buf[tx_index++] = instruction;
  }

  // Address bytes
  for (uint32_t i = 0; i < address_len; i++) {
    tx_buf[tx_index++] = (uint8_t)((address >> (8 * (address_len - 1 - i))) & 0xFF);
  }

  // Alternate bytes
  for (uint32_t i = 0; i < alternate_len; i++) {
    tx_buf[tx_index++] = (uint8_t)((alternate >> (8 * (alternate_len - 1 - i))) & 0xFF);
  }

  // Dummy bytes
  for (uint32_t i = 0; i < dummy; i++) {
    tx_buf[tx_index++] = 0x00; // Dummy bytes
  }

  // Data to write
  if (in_len > 0 && in_buf != NULL) {
    memcpy(&tx_buf[tx_index], in_buf, in_len);
    tx_index += in_len;
  }

  uint32_t total_length = tx_index;

  // If reading data, append dummy bytes to tx_buf to generate clock for reading
  if (out_len > 0) {
    memset(&tx_buf[tx_index], 0xFF, out_len); // Fill with dummy bytes
    total_length += out_len;
  }

  // Start SPI transaction
  nrf_gpio_pin_clear(W25QXX_SPI_CS_PIN); // CS low

  // Prepare transfer descriptor
  nrfx_spim_xfer_desc_t xfer_desc = {
    .p_tx_buffer = tx_buf,
    .tx_length = total_length,
    .p_rx_buffer = (out_len > 0) ? rx_buf : NULL,
    .rx_length = (out_len > 0) ? total_length : 0
  };

  spi_xfer_done = false;
  ret_code_t ret = nrfx_spim_xfer(&spim_instance, &xfer_desc, 0);
  if (ret != NRF_SUCCESS) {
    nrf_gpio_pin_set(W25QXX_SPI_CS_PIN); // CS high
    return 1; // Transfer failed
  }

  // Wait for transfer to complete
  while (!spi_xfer_done) {
    __WFE(); // Wait for event
  }

  // Copy received data to out_buf
  if (out_len > 0 && out_buf != NULL) {
    uint32_t data_start_index = tx_index; // Data starts after tx_index
    memcpy(out_buf, &rx_buf[tx_index], out_len);
  }

  nrf_gpio_pin_set(W25QXX_SPI_CS_PIN); // CS high

  return 0; // Success
}

/**
 * @brief     interface delay ms
 * @param[in] ms
 * @note      none
 */
void w25qxx_interface_delay_ms(uint32_t ms) {
  nrf_delay_ms(ms);
}

/**
 * @brief     interface delay us
 * @param[in] us
 * @note      none
 */
void w25qxx_interface_delay_us(uint32_t us) {
  nrf_delay_us(us);
}

/**
 * @brief     interface print format data
 * @param[in] fmt is the format data
 * @note      none
 */
void w25qxx_interface_debug_print(const char *const fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args); // Ensure standard output is set up (e.g., over UART)
  va_end(args);
}
