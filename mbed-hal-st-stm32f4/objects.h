/* mbed Microcontroller Library
 *******************************************************************************
 * Copyright (c) 2014, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */
#ifndef MBED_OBJECTS_H
#define MBED_OBJECTS_H

#include "cmsis.h"
#include "PortNames.h"
#include "PeripheralNames.h"
#include "PinNames.h"
#include "target_config.h"

// betzw
#include "dma_api.h"

#ifdef __cplusplus
extern "C" {
#endif

struct gpio_irq_s {
    IRQn_Type irq_n;
    uint32_t irq_index;
    uint32_t event;
    PinName pin;
};

struct port_s {
    PortName port;
    uint32_t mask;
    PinDirection direction;
    __IO uint32_t *reg_in;
    __IO uint32_t *reg_out;
};

struct analogin_s {
    ADCName adc;
    PinName pin;
    uint8_t channel;
};

struct dac_s {
    DACName dac;
    uint8_t channel;
};

struct serial_s {
    PinName pin_tx;
    PinName pin_rx;
    uint8_t module;
    uint32_t event;
    uint8_t char_match;
 };

struct spi_s {
    PinName pin_miso;
    PinName pin_mosi;
    PinName pin_sclk;
    uint32_t event;
    uint8_t module;
    uint8_t transfer_type;
};

struct i2s_s {
    PinName pin_data;
    PinName pin_sclk;
    PinName pin_wsel;
    PinName pin_fdpx;
    uint32_t event;
    uint8_t module;
    uint8_t transfer_type;
};

struct i2c_s {
    I2CName  i2c;
    uint32_t slave;
};

struct pwmout_s {
    PWMName pwm;
    PinName pin;
    uint32_t period;
    uint32_t pulse;
    uint8_t channel;
    uint8_t inverted;
};

struct sleep_s {
    TIM_HandleTypeDef TimMasterHandle;
};

struct dma_stream_s {
	DMA_Stream_TypeDef *dma_stream;
	IRQn_Type dma_stream_irq;
	uint32_t  channel_nr;
	uint32_t  channel_nr_fd;
	uint8_t busy;
};

struct dma_s {
    uint8_t dma_device;                                /**< DMA device */
    uint8_t dma_direction;                             /**< Primary DMA direction */
    const struct dma_stream_s *dma[NUM_OF_DIRECTIONS]; /**< Tx/Rx DMA devices */ // NOTE: do NOT touch contents!
};

#include "gpio_object.h"

#ifdef __cplusplus
}
#endif

#endif
