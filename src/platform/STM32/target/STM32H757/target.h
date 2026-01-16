/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * STM32H757 dual-core target
 * Betaflight runs on M4 core (Cortex-M4)
 * M7 core runs ChibiOS/other RTOS
 */

#pragma once

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "H757"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "Betaflight STM32H757"
#endif

#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3

#define USE_VCP

#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART7
#define USE_UART8

#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff
#define TARGET_IO_PORTG 0xffff
#define TARGET_IO_PORTH 0xffff

#define USE_I2C
#define I2C_FULL_RECONFIGURABILITY

#define USE_BEEPER

#define USE_SPI
#define SPI_FULL_RECONFIGURABILITY
#define USE_SPI_DMA_ENABLE_LATE

#define USE_ESCSERIAL

#define USE_ADC
#define USE_EXTI
#define USE_TIMER_UP_CONFIG

#define FLASH_PAGE_SIZE ((uint32_t)0x20000) // 128K sectors

#if defined(USE_LED_STRIP) && !defined(USE_LED_STRIP_CACHE_MGMT)
#define USE_LED_STRIP_CACHE_MGMT
#endif
