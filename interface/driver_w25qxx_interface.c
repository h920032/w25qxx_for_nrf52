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
 * @file      driver_w25qxx_interface.c
 * @brief     driver w25qxx interface for nrf52xxx source file
 * @version   1.1.0
 * @date      2024-12-04
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2021/07/15  <td>1.0      <td>Shifeng Li  <td>first upload
 * <tr><td>2024/12/04  <td>1.1      <td>Hugo Shih   <td>update interface
 * </table>
 */

#include "driver_w25qxx_interface.h"
#include "main.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrfx_spim.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h> // Added for memset and memcpy

/* SPI instance index */
#define W25QXX_SPI_INSTANCE 0 /**< SPIM instance to use */

/* Buffer sizes */
#define RX_BUFFER_SIZE 256 /**< Receive buffer size */
#define TX_BUFFER_SIZE 256 /**< Transmit buffer size */

/* SPIM instance */
static const nrfx_spim_t spim_instance = NRFX_SPIM_INSTANCE(W25QXX_SPI_INSTANCE);

/* Flag to indicate SPI transfer completion */
static volatile bool spi_xfer_done = false;

/* Transmit and receive buffers */
static uint8_t tx_buf[TX_BUFFER_SIZE]; /**< Transmit buffer */
static uint8_t rx_buf[RX_BUFFER_SIZE]; /**< Receive buffer */

/**
 * @brief SPI event handler called upon transfer completion.
 *
 * @param[in] p_event   Pointer to the SPIM event structure.
 * @param[in] p_context User-defined context (unused here).
 */
void spi_event_handler(nrfx_spim_evt_t const *p_event, void *p_context) {
  spi_xfer_done = true;
}

/**
 * @brief  Initializes the SPI interface for the W25QXX flash memory.
 * @return Status code
 *         - 0: Success
 *         - 1: SPI initialization failed
 */
uint8_t w25qxx_interface_spi_qspi_init(void) {
  nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG;

  /* Configure SPI pins */
  spim_config.ss_pin = NRFX_SPIM_PIN_NOT_USED; // CS pin handled manually
  spim_config.miso_pin = W25QXX_SPI_MISO_PIN;
  spim_config.mosi_pin = W25QXX_SPI_MOSI_PIN;
  spim_config.sck_pin = W25QXX_SPI_SCK_PIN;

  /* Configure SPI parameters */
  spim_config.frequency = NRF_SPIM_FREQ_8M; // Set frequency as needed
  spim_config.mode = NRF_SPIM_MODE_0;       // SPI mode 0 (CPOL = 0, CPHA = 0)
  spim_config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;

  /* Initialize the SPIM instance */
  ret_code_t ret = nrfx_spim_init(&spim_instance, &spim_config, spi_event_handler, NULL);
  if (ret != NRF_SUCCESS) {
    NRF_LOG_ERROR("SPI initialization failed with error: %d", ret);
    return 1; // Initialization failed
  }

  /* Configure CS pin as output and set it high (inactive) */
  nrf_gpio_cfg_output(W25QXX_SPI_CS_PIN);
  nrf_gpio_pin_set(W25QXX_SPI_CS_PIN);

  return 0; // Success
}

/**
 * @brief  Deinitializes the SPI interface.
 * @return Status code
 *         - 0: Success
 */
uint8_t w25qxx_interface_spi_qspi_deinit(void) {
  nrfx_spim_uninit(&spim_instance);
  return 0; // Success
}

/**
 * @brief      Performs SPI write/read operations with the W25QXX flash memory.
 *
 * @param[in]  instruction      Instruction byte to send.
 * @param[in]  instruction_line Unused (for compatibility).
 * @param[in]  address          Address value.
 * @param[in]  address_line     Unused (for compatibility).
 * @param[in]  address_len      Number of address bytes (0 to 4).
 * @param[in]  alternate        Alternate bytes value.
 * @param[in]  alternate_line   Unused (for compatibility).
 * @param[in]  alternate_len    Number of alternate bytes.
 * @param[in]  dummy            Number of dummy bytes to send.
 * @param[in]  *in_buf          Pointer to data to write (can be NULL).
 * @param[in]  in_len           Length of data to write.
 * @param[out] *out_buf         Pointer to buffer to store read data (can be NULL).
 * @param[in]  out_len          Length of data to read.
 * @param[in]  data_line        Unused (for compatibility).
 *
 * @return     Status code
 *             - 0: Success
 *             - 1: Transfer failed
 *
 * @note       This function handles both write and read operations over SPI.
 */
uint8_t w25qxx_interface_spi_qspi_write_read(uint8_t instruction, uint8_t instruction_line,
    uint32_t address, uint8_t address_line, uint8_t address_len,
    uint32_t alternate, uint8_t alternate_line, uint8_t alternate_len,
    uint8_t dummy, uint8_t *in_buf, uint32_t in_len,
    uint8_t *out_buf, uint32_t out_len, uint8_t data_line) {
  uint32_t tx_index = 0;
  ret_code_t ret;
  nrfx_spim_xfer_desc_t xfer_desc;

  /* Validate input parameters */
  if (address_len > 4 || alternate_len > 4) {
    NRF_LOG_ERROR("Invalid address_len or alternate_len");
    return 1; // Invalid parameters
  }
  if ((in_len > 0 && in_buf == NULL) || (out_len > 0 && out_buf == NULL)) {
    NRF_LOG_ERROR("Invalid buffer pointers");
    return 1; // Invalid buffer pointers
  }

  /* Build the transmit buffer with instruction, address, alternate, and dummy bytes */

  /* Add instruction byte if provided */
  if (instruction != 0x00) {
    if (tx_index >= TX_BUFFER_SIZE) {
      NRF_LOG_ERROR("Transmit buffer overflow");
      return 1; // Buffer overflow
    }
    tx_buf[tx_index++] = instruction;
  }

  /* Append address bytes (big-endian) */
  for (uint32_t i = 0; i < address_len; i++) {
    if (tx_index >= TX_BUFFER_SIZE) {
      NRF_LOG_ERROR("Transmit buffer overflow during address append");
      return 1; // Buffer overflow
    }
    tx_buf[tx_index++] = (uint8_t)((address >> (8 * (address_len - 1 - i))) & 0xFF);
  }

  /* Append alternate bytes (big-endian) */
  for (uint32_t i = 0; i < alternate_len; i++) {
    if (tx_index >= TX_BUFFER_SIZE) {
      NRF_LOG_ERROR("Transmit buffer overflow during alternate append");
      return 1; // Buffer overflow
    }
    tx_buf[tx_index++] = (uint8_t)((alternate >> (8 * (alternate_len - 1 - i))) & 0xFF);
  }

  /* Append dummy bytes */
  for (uint32_t i = 0; i < dummy; i++) {
    if (tx_index >= TX_BUFFER_SIZE) {
      NRF_LOG_ERROR("Transmit buffer overflow during dummy bytes append");
      return 1; // Buffer overflow
    }
    tx_buf[tx_index++] = 0x00; // Dummy byte
  }

  /* Set CS low to start the SPI transaction */
  nrf_gpio_pin_clear(W25QXX_SPI_CS_PIN); // CS low

  /* Wait for transfer to complete with timeout to prevent infinite loop */
  uint32_t timeout = 1000000; // Adjust as necessary

  /* Handle data write if in_len > 0 */
  if (in_len > 0 && in_buf != NULL) {
    uint32_t bytes_remaining = in_len;
    uint32_t offset = 0;

    /* Write data in chunks */
    while (bytes_remaining > 0) {
      uint32_t chunk_size = (bytes_remaining > TX_BUFFER_SIZE - 1) ? TX_BUFFER_SIZE - 1 : bytes_remaining;

      /* Copy data to tx_buf */
      memcpy(tx_buf, in_buf + offset, chunk_size);

      xfer_desc.p_tx_buffer = tx_buf;
      xfer_desc.tx_length = chunk_size;
      xfer_desc.p_rx_buffer = NULL;
      xfer_desc.rx_length = 0;

      spi_xfer_done = false;
      ret = nrfx_spim_xfer(&spim_instance, &xfer_desc, 0);
      if (ret != NRF_SUCCESS) {
        nrf_gpio_pin_set(W25QXX_SPI_CS_PIN); // CS high
        NRF_LOG_ERROR("SPI write transfer failed with error: %d", ret);
        return 1; // Transfer failed
      }

      /* Wait for transfer to complete with timeout */
      timeout = 1000000; // Reset timeout
      while (!spi_xfer_done && timeout--) {
        __WFE(); // Wait for event
      }
      if (timeout == 0) {
        nrf_gpio_pin_set(W25QXX_SPI_CS_PIN); // CS high
        NRF_LOG_ERROR("SPI write transfer timed out");
        return 1; // Timeout
      }

      offset += chunk_size;
      bytes_remaining -= chunk_size;
    }
  }

  /* Handle data read if out_len > 0 */
  if (out_len > 0 && out_buf != NULL) {
    uint32_t bytes_remaining = out_len;
    uint32_t offset = 0;

    /* Read data in chunks */
    while (bytes_remaining > 0) {
      uint32_t chunk_size = (bytes_remaining > RX_BUFFER_SIZE - 1) ? RX_BUFFER_SIZE - 1 : bytes_remaining;

      /* Prepare tx_buf with dummy bytes to generate clock cycles */
      memset(tx_buf, 0xFF, chunk_size); // Dummy bytes

      xfer_desc.p_tx_buffer = NULL;
      xfer_desc.tx_length = 0;
      xfer_desc.p_rx_buffer = rx_buf;
      xfer_desc.rx_length = chunk_size;

      spi_xfer_done = false;
      ret = nrfx_spim_xfer(&spim_instance, &xfer_desc, 0);
      if (ret != NRF_SUCCESS) {
        nrf_gpio_pin_set(W25QXX_SPI_CS_PIN); // CS high
        NRF_LOG_ERROR("SPI read transfer failed with error: %d", ret);
        return 1; // Transfer failed
      }

      /* Wait for transfer to complete with timeout */
      timeout = 1000000; // Reset timeout
      while (!spi_xfer_done && timeout--) {
        __WFE(); // Wait for event
      }
      if (timeout == 0) {
        nrf_gpio_pin_set(W25QXX_SPI_CS_PIN); // CS high
        NRF_LOG_ERROR("SPI read transfer timed out");
        return 1; // Timeout
      }

      /* Copy received data to out_buf */
      memcpy(out_buf + offset, rx_buf, chunk_size);

      offset += chunk_size;
      bytes_remaining -= chunk_size;
    }
  }

  /* Set CS high to end the SPI transaction */
  nrf_gpio_pin_set(W25QXX_SPI_CS_PIN); // CS high

  return 0; // Success
}

/**
 * @brief     Delays execution for the specified number of milliseconds.
 * @param[in] ms Number of milliseconds to delay.
 */
void w25qxx_interface_delay_ms(uint32_t ms) {
  /* Check for maximum delay value */
  if (ms > UINT32_MAX / 1000) {
    NRF_LOG_WARNING("Requested delay exceeds maximum value");
    ms = UINT32_MAX / 1000;
  }
  nrf_delay_ms(ms);
}

/**
 * @brief     Delays execution for the specified number of microseconds.
 * @param[in] us Number of microseconds to delay.
 */
void w25qxx_interface_delay_us(uint32_t us) {
  /* Check for maximum delay value */
  if (us > UINT32_MAX / 1) {
    NRF_LOG_WARNING("Requested delay exceeds maximum value");
    us = UINT32_MAX / 1;
  }
  nrf_delay_us(us);
}

/**
 * @brief     Prints formatted debug information.
 * @param[in] fmt Format string (printf-style).
 */
void w25qxx_interface_debug_print(const char *const fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args); // Ensure standard output is set up (e.g., over UART)
  va_end(args);
}