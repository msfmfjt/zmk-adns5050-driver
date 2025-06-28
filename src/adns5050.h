#pragma once

#include <zephyr/drivers/sensor.h>
#include "pixart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Timings (in us) used in SPI communication for ADNS5050 */
#define T_NCS_SCLK 1     /* 120 ns (rounded to 1us) */
#define T_SCLK_NCS_WR 10 /* 10 us */
#define T_SRAD 4         /* 4 us */
#define T_SRX 1          /* 250 ns (rounded to 1 us) */
#define T_SWX 30         /* SWW: 30 us, SWR: 20 us */

/* Sensor registers (addresses) for ADNS5050 */
#define ADNS5050_REG_PRODUCT_ID 0x00
#define ADNS5050_REG_REVISION_ID 0x01
#define ADNS5050_REG_MOTION 0x02
#define ADNS5050_REG_DELTA_X 0x03
#define ADNS5050_REG_DELTA_Y 0x04
#define ADNS5050_REG_SQUAL 0x05
#define ADNS5050_REG_SHUTTER_UPPER 0x06
#define ADNS5050_REG_SHUTTER_LOWER 0x07
#define ADNS5050_REG_PIXEL_SUM 0x08
#define ADNS5050_REG_MAXIMUM_PIXEL 0x09
#define ADNS5050_REG_MINIMUM_PIXEL 0x0A
#define ADNS5050_REG_PIXEL_GRAB 0x0B
#define ADNS5050_REG_MOUSE_CONTROL 0x0D
#define ADNS5050_REG_MOUSE_CONTROL2 0x19
#define ADNS5050_REG_LED_DC_MODE 0x22
#define ADNS5050_REG_CHIP_RESET 0x3A
#define ADNS5050_REG_PRODUCT_ID2 0x3E
#define ADNS5050_REG_INV_REV_ID 0x3F
#define ADNS5050_REG_MOTION_BURST 0x63

/* Sensor identification values */
#define ADNS5050_PRODUCT_ID 0x12

/* Reset command */
#define ADNS5050_RESET_CMD 0x5A

/* write command bit position */
#define SPI_WRITE_BIT BIT(7)

/* ADNS5050 specific settings */
#define ADNS5050_RESOLUTION_COUNT_MAX 1600
#define ADNS5050_RESOLUTION_COUNT_MIN 125

/* Helper macros used to convert sensor values. */
#define ADNS5050_SVALUE_TO_CPI(svalue) ((uint32_t)(svalue).val1)
#define ADNS5050_SVALUE_TO_TIME(svalue) ((uint32_t)(svalue).val1)

#ifdef CONFIG_ADNS5050_INVERT_SCROLL_X
#define ADNS5050_SCROLL_X_NEGATIVE 1
#define ADNS5050_SCROLL_X_POSITIVE -1
#else
#define ADNS5050_SCROLL_X_NEGATIVE -1
#define ADNS5050_SCROLL_X_POSITIVE 1
#endif

#ifdef CONFIG_ADNS5050_INVERT_SCROLL_Y
#define ADNS5050_SCROLL_Y_NEGATIVE 1
#define ADNS5050_SCROLL_Y_POSITIVE -1
#else
#define ADNS5050_SCROLL_Y_NEGATIVE -1
#define ADNS5050_SCROLL_Y_POSITIVE 1
#endif

#ifdef __cplusplus
}
#endif
