/*

Copyright 2011-2018 Stratify Labs, Inc

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

 */


#include <sys/lock.h>
#include <fcntl.h>
#include <errno.h>
#include <mcu/mcu.h>
#include <mcu/debug.h>
#include <mcu/periph.h>
#include <device/sys.h>
#include <device/uartfifo.h>
#include <device/usbfifo.h>
#include <device/fifo.h>
#include <device/cfifo.h>
#include <device/stream_ffifo.h>
#include <device/sys.h>
#include <sos/link.h>
#include <sos/fs/sysfs.h>
#include <sos/fs/appfs.h>
#include <sos/fs/devfs.h>
#include <sos/fs/sffs.h>
#include <sos/sos.h>

#include "config.h"
#include "link_config.h"

/*
 *
 * Audio is 8000 bytes/second
 *
 * Flash memory 128 2K pages
 * 22830 for bootloader (11 pages)
 * 118496 for OS (58 pages) - 6 pages at the end == 12KB ~1.5 seconds
 *
 * 45936 for Application (22 pages)
 *
 * 9 pages = 18KB = 2.3 seconds
 *
 *
 *
 *
 *
 *
 *
 */


//--------------------------------------------Stratify OS Configuration-------------------------------------------------
const sos_board_config_t sos_board_config = {
    .clk_usecond_tmr = SOS_BOARD_TMR, //TIM2 -- 32 bit timer
    .task_total = SOS_BOARD_TASK_TOTAL,
    .stdin_dev = "/dev/stdio-in" ,
    .stdout_dev = "/dev/stdio-out",
    .stderr_dev = "/dev/stdio-out",
    .o_sys_flags = SYS_FLAG_IS_STDIO_FIFO | SYS_FLAG_IS_TRACE,
    .sys_name = SOS_BOARD_NAME,
    .sys_version = SOS_BOARD_VERSION,
	 .sys_id = SOS_BOARD_ID,
    .sys_memory_size = SOS_BOARD_SYSTEM_MEMORY_SIZE,
    .start = sos_default_thread,
    .start_args = &link_transport,
    .start_stack_size = SOS_DEFAULT_START_STACK_SIZE,
    .socket_api = 0,
    .request = 0,
    .trace_dev = "/dev/trace",
    .trace_event = SOS_BOARD_TRACE_EVENT,
    .git_hash = SOS_GIT_HASH
};

//This declares the task tables required by Stratify OS for applications and threads
SOS_DECLARE_TASK_TABLE(SOS_BOARD_TASK_TOTAL);

//--------------------------------------------Device Filesystem-------------------------------------------------


/*
 * Defaults configurations
 *
 * This provides the default pin assignments and settings for peripherals. If
 * the defaults are not provided, the application must specify them.
 *
 * Defaults should be added for peripherals that are dedicated for use on the
 * board. For example, if a UART has an external connection and label on the
 * board, the BSP should provide the default configuration.
 *
 *
 *
 */
UARTFIFO_DECLARE_CONFIG_STATE(uart0_fifo, 1024, 1,
                              UART_FLAG_SET_LINE_CODING_DEFAULT, 8, 115200,
                              0, 2,
                              0, 3,
                              0xff, 0xff,
                              0xff, 0xff);

I2C_DECLARE_CONFIG_MASTER(i2c0,
                          I2C_FLAG_SET_MASTER,
                          100000,
								  1, 9, //SDA PB8
								  1, 8 //SCL PB9
                          );

I2C_DECLARE_CONFIG_MASTER(i2c1,
								  I2C_FLAG_SET_MASTER,
								  100000,
								  1, 11, //SDA PB11
								  1, 10 //SCL PB10
								  );

SPI_DECLARE_CONFIG(spi2,
                   SPI_FLAG_SET_MASTER |
                   SPI_FLAG_IS_FORMAT_SPI |
                   SPI_FLAG_IS_MODE0,
                   1000000,
                   8,
                   2, 11, //SPI MISO PC11
                   2, 12, //SPI MOSI PC12
                   2, 10, //SPI SCLK PC10
                   0xff, 0xff //SPI CS (not used)
                   );

const stm32_dac_dma_config_t dac0_dma_config = {
    .dac_config = {
        .attr  = {
            .o_flags = DAC_FLAG_SET_CONVERTER |
            DAC_FLAG_SET_CHANNELS |
            DAC_FLAG_IS_RIGHT_JUSTIFIED |
            DAC_FLAG_IS_TRIGGER_TMR,
            .pin_assignment = {
                .channel[0] = {0, 4},
                .channel[1] = {0xff, 0xff},
                .channel[2] = {0xff, 0xff},
                .channel[3] = {0xff, 0xff}
            },
            .freq = 0,
            .width = 12,
            .trigger = {6, 0}, //trigger is TIM7
        },
        .reference_mv = 3300
    },
    .dma_config = {
        .dma_number = STM32_DMA1,
        .stream_number = 2, //2, //channel on STM32L4 This is CHANNEL3 numbering is 0,1,2 versus CHANNEL1, CHANNEL2, ...
        .channel_number = 6, //6, //request on STM32L4
        .priority = STM32_DMA_PRIORITY_LOW
    }
};

const stm32_dac_dma_config_t dac1_dma_config = {
    .dac_config = {
        .attr  = {
            .o_flags = DAC_FLAG_SET_CONVERTER |
            DAC_FLAG_SET_CHANNELS |
            DAC_FLAG_IS_RIGHT_JUSTIFIED |
            DAC_FLAG_IS_TRIGGER_TMR,
            .pin_assignment = {
                .channel[0] = {0, 5},
                .channel[1] = {0xff, 0xff},
                .channel[2] = {0xff, 0xff},
                .channel[3] = {0xff, 0xff}
            },
            .freq = 0,
            .width = 12,
            .trigger = {6, 0}, //trigger is TIM7
        },
        .reference_mv = 3300
    },
    .dma_config = {
        .dma_number = STM32_DMA1,
        .stream_number = 3, //2, //channel on STM32L4 This is CHANNEL3 numbering is 0,1,2 versus CHANNEL1, CHANNEL2, ...
        .channel_number = 5, //6, //request on STM32L4
        .priority = STM32_DMA_PRIORITY_LOW,
        .o_flags = 0
    }
};

#if 1
const devfs_device_t dac_dma = DEVFS_DEVICE("dac0", mcu_dac_dma, 0, &dac0_dma_config, 0, 0666, SOS_USER_ROOT, S_IFCHR);
#else
const devfs_device_t dac_dma = DEVFS_DEVICE("dac1", mcu_dac_dma, 1, &dac1_dma_config, 0, 0666, SOS_USER_ROOT, S_IFCHR);
#endif

STREAM_FFIFO_DECLARE_CONFIG_STATE_TX_ONLY(dac_stream_ffifo, 512, 2, &dac_dma, 0);


#if 0
//SAI2_a unit - master receiver port 0 is SAI1A, 1 is SAI1B, 2 is SAI2A
const stm32_sai_dma_config_t sai2_dma_config = {
    .i2s_config = {
        .attr = {
            .o_flags = I2S_FLAG_SET_MASTER |
            I2S_FLAG_IS_STEREO |
            I2S_FLAG_IS_WIDTH_24 |
            I2S_FLAG_IS_RECEIVER,
            .freq = 16000,
            .pin_assignment = {
                .ws = {1, 12},
                .sck = {1, 13},
                .sdout = {0xff, 0xff},
                .sdin = {1, 15},
                .mck = {0xff, 0xff}
            }
        }
    },
    .dma_config = {
        .dma_number = STM32_DMA1,
        .stream_number = 5, //channel 6, //channel on STM32L4 This is CHANNEL3 numbering is 0,1,2 versus CHANNEL1, CHANNEL2, ...
        .channel_number = 1, //request 1, //request on STM32L4 request starts with numbering 0 REQUEST0
        .priority = STM32_DMA_PRIORITY_LOW,
        .o_flags = STM32_DMA_FLAG_IS_CIRCULAR |
        STM32_DMA_FLAG_IS_MEMORY_WORD |
        STM32_DMA_FLAG_IS_PERIPH_WORD
    }
};

const devfs_device_t sai_dma = DEVFS_DEVICE("sai_dma", mcu_sai_dma, 2, &sai2_dma_config, 0, 0666, SOS_USER_ROOT, S_IFCHR);
STREAM_FFIFO_DECLARE_CONFIG_STATE_RX_ONLY(mic_stream_ffifo, 512, 2, &sai_dma, 0);

#endif



FIFO_DECLARE_CONFIG_STATE(stdio_in, SOS_BOARD_STDIO_BUFFER_SIZE);
FIFO_DECLARE_CONFIG_STATE(stdio_out, SOS_BOARD_STDIO_BUFFER_SIZE);
CFIFO_DECLARE_CONFIG_STATE_4(board_fifo, 256);

/* This is the list of devices that will show up in the /dev folder.
 */
const devfs_device_t devfs_list[] = {
    //System devices
    DEVFS_DEVICE("trace", ffifo, 0, &board_trace_config, &board_trace_state, 0666, SOS_USER_ROOT, S_IFCHR),
    DEVFS_DEVICE("fifo", cfifo, 0, &board_fifo_config, &board_fifo_state, 0666, SOS_USER_ROOT, S_IFCHR),
    DEVFS_DEVICE("stdio-out", fifo, 0, &stdio_out_config, &stdio_out_state, 0666, SOS_USER_ROOT, S_IFCHR),
    DEVFS_DEVICE("stdio-in", fifo, 0, &stdio_in_config, &stdio_in_state, 0666, SOS_USER_ROOT, S_IFCHR),
    DEVFS_DEVICE("link-phy-usb", usbfifo, 0, &sos_link_transport_usb_fifo_cfg, &sos_link_transport_usb_fifo_state, 0666, SOS_USER_ROOT, S_IFCHR),
    DEVFS_DEVICE("sys", sys, 0, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),

    //MCU peripherals
    DEVFS_DEVICE("core", mcu_core, 0, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),
    DEVFS_DEVICE("core0", mcu_core, 0, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),

    DEVFS_DEVICE("dac0", mcu_dac_dma, 0, &dac0_dma_config, 0, 0666, SOS_USER_ROOT, S_IFCHR), //direct access to the DAC for testing
    DEVFS_DEVICE("dac1", mcu_dac_dma, 1, &dac1_dma_config, 0, 0666, SOS_USER_ROOT, S_IFCHR), //direct access to the DAC for testing

	 DEVFS_DEVICE("i2c0", mcu_i2c, 0, &i2c0_config, 0, 0666, SOS_USER_ROOT, S_IFCHR),
    DEVFS_DEVICE("i2c1", mcu_i2c, 1, &i2c1_config, 0, 0666, SOS_USER_ROOT, S_IFCHR),
    DEVFS_DEVICE("i2c2", mcu_i2c, 2, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),
    DEVFS_DEVICE("i2c3", mcu_i2c, 3, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),

    DEVFS_DEVICE("pio0", mcu_pio, 0, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //GPIOA
    DEVFS_DEVICE("pio1", mcu_pio, 1, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //GPIOB
    DEVFS_DEVICE("pio2", mcu_pio, 2, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //GPIOC
    DEVFS_DEVICE("pio3", mcu_pio, 3, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //GPIOD
    DEVFS_DEVICE("pio4", mcu_pio, 4, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //GPIOE
    DEVFS_DEVICE("pio5", mcu_pio, 5, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //GPIOF
    DEVFS_DEVICE("pio6", mcu_pio, 6, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //GPIOG
    DEVFS_DEVICE("pio7", mcu_pio, 7, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //GPIOH

    DEVFS_DEVICE("spi0", mcu_spi, 0, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),
    DEVFS_DEVICE("spi1", mcu_spi, 1, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),
    DEVFS_DEVICE("spi2", mcu_spi, 2, &spi2_config, 0, 0666, SOS_USER_ROOT, S_IFCHR),
    DEVFS_DEVICE("spi3", mcu_spi, 3, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),

    DEVFS_DEVICE("tmr0", mcu_tmr, 0, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //TIM1
    DEVFS_DEVICE("tmr1", mcu_tmr, 1, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //TIM2
    DEVFS_DEVICE("tmr2", mcu_tmr, 2, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),
    DEVFS_DEVICE("tmr3", mcu_tmr, 3, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),
    DEVFS_DEVICE("tmr4", mcu_tmr, 4, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),
    DEVFS_DEVICE("tmr5", mcu_tmr, 5, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),
    DEVFS_DEVICE("tmr6", mcu_tmr, 6, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //TIM7
    DEVFS_DEVICE("tmr7", mcu_tmr, 7, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //TIM8
    //Does this chip have more timers?

    DEVFS_DEVICE("uart0", uartfifo, 0, &uart0_fifo_config, &uart0_fifo_state, 0666, SOS_USER_ROOT, S_IFCHR),
    DEVFS_TERMINATOR
};


//--------------------------------------------Root Filesystem---------------------------------------------------

/*
 * This is the root filesystem that determines what is mounted at /.
 *
 * The default is /app (for installing and running applciations in RAM and flash) and /dev which
 * provides the device tree defined above.
 *
 * Additional filesystems (such as FatFs) can be added if the hardware and drivers
 * are provided by the board.
 *
 */

const devfs_device_t mem0 = DEVFS_DEVICE("mem0", mcu_mem, 0, 0, 0, 0666, SOS_USER_ROOT, S_IFBLK);
const sysfs_t sysfs_list[] = {
	 APPFS_MOUNT("/app", &mem0, 0777, SYSFS_ROOT), //the folder for ram/flash applications
	 DEVFS_MOUNT("/dev", devfs_list, 0777, SYSFS_ROOT), //the list of devices
	 SYSFS_MOUNT("/", sysfs_list, 0777, SYSFS_ROOT), //the root filesystem (must be last)
    SYSFS_TERMINATOR
};


