/*
 * Copyright (c) 2024 Your Name
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_adns5050

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(adns5050, CONFIG_ZMK_LOG_LEVEL);

/* ADNS5050 Registers */
#define ADNS5050_REG_PRODUCT_ID           0x00
#define ADNS5050_REG_REVISION_ID          0x01
#define ADNS5050_REG_MOTION               0x02
#define ADNS5050_REG_DELTA_X              0x03
#define ADNS5050_REG_DELTA_Y              0x04
#define ADNS5050_REG_SQUAL                0x05
#define ADNS5050_REG_SHUTTER_UPPER        0x06
#define ADNS5050_REG_SHUTTER_LOWER        0x07
#define ADNS5050_REG_MAXIMUM_PIXEL        0x08
#define ADNS5050_REG_PIXEL_SUM            0x09
#define ADNS5050_REG_MINIMUM_PIXEL        0x0A
#define ADNS5050_REG_PIXEL_GRAB           0x0B
#define ADNS5050_REG_MOUSE_CONTROL        0x0D
#define ADNS5050_REG_MOUSE_CONTROL2       0x19
#define ADNS5050_REG_LED_DC_MODE          0x22
#define ADNS5050_REG_CHIP_RESET           0x3A
#define ADNS5050_REG_PRODUCT_ID2          0x3E
#define ADNS5050_REG_INV_REV_ID           0x3F
#define ADNS5050_REG_MOTION_BURST         0x63

/* Configuration values */
#define ADNS5050_PRODUCT_ID               0x12
#define ADNS5050_RESET_VALUE              0x5A

/* Timing delays (microseconds) */
#define ADNS5050_TIMINGS_RESET            250   /* tRESET - Reset pulse width (ns) */
#define ADNS5050_TIMINGS_MOT_RST          50000 /* tMOT-RST - Motion delay after reset (us) */
#define ADNS5050_TIMINGS_SWW              30    /* tSWW - Time between write commands */
#define ADNS5050_TIMINGS_SWR              20    /* tSWR - Time between write and read */
#define ADNS5050_TIMINGS_SRW              1     /* tSRW - Time between read and subsequent commands */
#define ADNS5050_TIMINGS_SRR              1     /* tSRR - Time between read commands */
#define ADNS5050_TIMINGS_SRAD             10    /* tSRAD - Read address-data delay (max) */
#define ADNS5050_TIMINGS_BEXIT            1     /* tBEXIT - NCS inactive after motion burst */

struct adns5050_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec motion_gpio;
    struct gpio_dt_spec reset_gpio;
    uint16_t cpi;
    bool invert_x;
    bool invert_y;
};

struct adns5050_data {
    const struct device *dev;
    struct k_work motion_work;
    struct gpio_callback motion_cb;
    int16_t x;
    int16_t y;
};

static inline int adns5050_read_reg(const struct device *dev, uint8_t reg, uint8_t *data)
{
    const struct adns5050_config *cfg = dev->config;
    int ret;
    
    uint8_t tx_buf[2] = {reg & 0x7F, 0}; /* Clear MSB for read */
    const struct spi_buf tx_bufs[] = {
        {.buf = tx_buf, .len = sizeof(tx_buf)}
    };
    const struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count = ARRAY_SIZE(tx_bufs)
    };
    
    uint8_t rx_buf[2];
    const struct spi_buf rx_bufs[] = {
        {.buf = rx_buf, .len = sizeof(rx_buf)}
    };
    const struct spi_buf_set rx = {
        .buffers = rx_bufs,
        .count = ARRAY_SIZE(rx_bufs)
    };
    
    ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
    if (ret < 0) {
        LOG_ERR("Failed to read register 0x%02x: %d", reg, ret);
        return ret;
    }
    
    *data = rx_buf[1];
    
    k_sleep(K_USEC(ADNS5050_TIMINGS_SRAD));
    
    return 0;
}

static inline int adns5050_write_reg(const struct device *dev, uint8_t reg, uint8_t data)
{
    const struct adns5050_config *cfg = dev->config;
    int ret;
    
    uint8_t tx_buf[2] = {reg | 0x80, data}; /* Set MSB for write */
    const struct spi_buf tx_bufs[] = {
        {.buf = tx_buf, .len = sizeof(tx_buf)}
    };
    const struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count = ARRAY_SIZE(tx_bufs)
    };
    
    ret = spi_write_dt(&cfg->spi, &tx);
    if (ret < 0) {
        LOG_ERR("Failed to write register 0x%02x: %d", reg, ret);
        return ret;
    }
    
    k_sleep(K_USEC(ADNS5050_TIMINGS_SWW));
    
    return 0;
}

static int adns5050_motion_burst_read(const struct device *dev, int16_t *x, int16_t *y)
{
    const struct adns5050_config *cfg = dev->config;
    int ret;
    uint8_t motion;
    
    /* Check motion status */
    ret = adns5050_read_reg(dev, ADNS5050_REG_MOTION, &motion);
    if (ret < 0) {
        return ret;
    }
    
    if (!(motion & 0x80)) {
        /* No motion */
        *x = 0;
        *y = 0;
        return 0;
    }
    
    /* Motion burst read */
    uint8_t tx_buf[4] = {ADNS5050_REG_MOTION_BURST & 0x7F, 0, 0, 0};
    const struct spi_buf tx_bufs[] = {
        {.buf = tx_buf, .len = sizeof(tx_buf)}
    };
    const struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count = ARRAY_SIZE(tx_bufs)
    };
    
    uint8_t rx_buf[4];
    const struct spi_buf rx_bufs[] = {
        {.buf = rx_buf, .len = sizeof(rx_buf)}
    };
    const struct spi_buf_set rx = {
        .buffers = rx_bufs,
        .count = ARRAY_SIZE(rx_bufs)
    };
    
    ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
    if (ret < 0) {
        LOG_ERR("Failed to read motion burst: %d", ret);
        return ret;
    }
    
    /* rx_buf[0] = dummy, rx_buf[1] = motion, rx_buf[2] = dx, rx_buf[3] = dy */
    *x = (int8_t)rx_buf[2];
    *y = (int8_t)rx_buf[3];
    
    /* Apply inversion if configured */
    if (cfg->invert_x) {
        *x = -*x;
    }
    if (cfg->invert_y) {
        *y = -*y;
    }
    
    k_sleep(K_USEC(ADNS5050_TIMINGS_SRAD_MOTBR));
    
    return 0;
}

static void adns5050_motion_work_handler(struct k_work *work)
{
    struct adns5050_data *data = CONTAINER_OF(work, struct adns5050_data, motion_work);
    const struct device *dev = data->dev;
    const struct adns5050_config *cfg = dev->config;
    int16_t x, y;
    int ret;
    
    /* Read motion data */
    ret = adns5050_motion_burst_read(dev, &x, &y);
    if (ret < 0) {
        LOG_ERR("Failed to read motion data: %d", ret);
        return;
    }
    
    /* Report motion if any */
    if (x != 0 || y != 0) {
        input_report_rel(dev, INPUT_REL_X, x, true, K_FOREVER);
        input_report_rel(dev, INPUT_REL_Y, y, true, K_FOREVER);
    }
    
    /* Re-enable interrupt */
    gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}

static void adns5050_motion_interrupt(const struct device *gpio_dev, struct gpio_callback *cb,
                                      uint32_t pins)
{
    struct adns5050_data *data = CONTAINER_OF(cb, struct adns5050_data, motion_cb);
    const struct device *dev = data->dev;
    const struct adns5050_config *cfg = dev->config;
    
    /* Disable interrupt to prevent multiple triggers */
    gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_DISABLE);
    
    /* Schedule work to read motion data */
    k_work_submit(&data->motion_work);
}

static int adns5050_init(const struct device *dev)
{
    const struct adns5050_config *cfg = dev->config;
    struct adns5050_data *data = dev->data;
    int ret;
    uint8_t val;
    
    data->dev = dev;
    
    /* Initialize SPI */
    if (!spi_is_ready_dt(&cfg->spi)) {
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }
    
    /* Initialize reset GPIO if available */
    if (cfg->reset_gpio.port != NULL) {
        if (!gpio_is_ready_dt(&cfg->reset_gpio)) {
            LOG_ERR("Reset GPIO not ready");
            return -ENODEV;
        }
        
        ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure reset GPIO: %d", ret);
            return ret;
        }
        
        /* Perform hardware reset */
        gpio_pin_set_dt(&cfg->reset_gpio, 1);
        k_sleep(K_USEC(10));
        gpio_pin_set_dt(&cfg->reset_gpio, 0);
        k_sleep(K_MSEC(40));
    }
    
    /* Software reset */
    ret = adns5050_write_reg(dev, ADNS5050_REG_CHIP_RESET, ADNS5050_RESET_VALUE);
    if (ret < 0) {
        LOG_ERR("Failed to reset chip: %d", ret);
        return ret;
    }
    k_sleep(K_MSEC(55));
    
    /* Read and verify product ID */
    ret = adns5050_read_reg(dev, ADNS5050_REG_PRODUCT_ID, &val);
    if (ret < 0) {
        LOG_ERR("Failed to read product ID: %d", ret);
        return ret;
    }
    
    if (val != ADNS5050_PRODUCT_ID) {
        LOG_ERR("Invalid product ID: 0x%02x (expected 0x%02x)", val, ADNS5050_PRODUCT_ID);
        return -ENODEV;
    }
    
    LOG_INF("ADNS5050 detected, product ID: 0x%02x", val);
    
    /* Configure CPI (counts per inch) */
    if (cfg->cpi != 500 && cfg->cpi != 1000) {
        /* Use Mouse_Control2 register for other resolutions */
        uint8_t res_val = 0x80; /* Enable RES_EN bit */
        
        switch (cfg->cpi) {
        case 125:
            res_val |= 0x01;
            break;
        case 250:
            res_val |= 0x02;
            break;
        case 375:
            res_val |= 0x03;
            break;
        case 625:
            res_val |= 0x05;
            break;
        case 750:
            res_val |= 0x06;
            break;
        case 875:
            res_val |= 0x07;
            break;
        case 1125:
            res_val |= 0x09;
            break;
        case 1250:
            res_val |= 0x0A;
            break;
        case 1375:
            res_val |= 0x0B;
            break;
        default:
            LOG_WRN("Unsupported CPI value %d, using default 500", cfg->cpi);
            break;
        }
        
        if (res_val != 0x80) {
            ret = adns5050_write_reg(dev, ADNS5050_REG_MOUSE_CONTROL2, res_val);
            if (ret < 0) {
                LOG_ERR("Failed to set CPI: %d", ret);
                return ret;
            }
        }
    } else if (cfg->cpi == 1000) {
        /* Use Mouse_Control register for 1000 CPI */
        ret = adns5050_write_reg(dev, ADNS5050_REG_MOUSE_CONTROL, 0x01);
        if (ret < 0) {
            LOG_ERR("Failed to set 1000 CPI: %d", ret);
            return ret;
        }
    }
    /* Default 500 CPI requires no configuration */
    
    /* Initialize motion interrupt if available */
    if (cfg->motion_gpio.port != NULL) {
        if (!gpio_is_ready_dt(&cfg->motion_gpio)) {
            LOG_ERR("Motion GPIO not ready");
            return -ENODEV;
        }
        
        ret = gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure motion GPIO: %d", ret);
            return ret;
        }
        
        /* Initialize work queue for motion handling */
        k_work_init(&data->motion_work, adns5050_motion_work_handler);
        
        /* Setup interrupt callback */
        gpio_init_callback(&data->motion_cb, adns5050_motion_interrupt, 
                          BIT(cfg->motion_gpio.pin));
        ret = gpio_add_callback(cfg->motion_gpio.port, &data->motion_cb);
        if (ret < 0) {
            LOG_ERR("Failed to add motion callback: %d", ret);
            return ret;
        }
        
        ret = gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure motion interrupt: %d", ret);
            return ret;
        }
    }
    
    LOG_INF("ADNS5050 initialized successfully");
    
    return 0;
}

#ifdef CONFIG_PM_DEVICE
static int adns5050_pm_action(const struct device *dev, enum pm_device_action action)
{
    int ret;
    
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        /* Put device in power-down mode */
        ret = adns5050_write_reg(dev, ADNS5050_REG_MOUSE_CONTROL, 0x10);
        if (ret < 0) {
            LOG_ERR("Failed to enter power-down mode: %d", ret);
            return ret;
        }
        break;
        
    case PM_DEVICE_ACTION_RESUME:
        /* Wake up device */
        ret = adns5050_write_reg(dev, ADNS5050_REG_MOUSE_CONTROL, 0x00);
        if (ret < 0) {
            LOG_ERR("Failed to exit power-down mode: %d", ret);
            return ret;
        }
        k_sleep(K_MSEC(55));
        break;
        
    default:
        return -ENOTSUP;
    }
    
    return 0;
}
#endif

#define ADNS5050_INIT(n)                                                       \
    static struct adns5050_data adns5050_data_##n;                           \
                                                                              \
    static const struct adns5050_config adns5050_config_##n = {              \
        .spi = SPI_DT_SPEC_INST_GET(n, SPI_OP_MODE_MASTER |                 \
                                       SPI_MODE_CPOL | SPI_MODE_CPHA |      \
                                       SPI_WORD_SET(8), 0),                  \
        .motion_gpio = GPIO_DT_SPEC_INST_GET_OR(n, motion_gpios, {0}),      \
        .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),        \
        .cpi = DT_INST_PROP(n, cpi),                                        \
        .invert_x = DT_INST_PROP(n, invert_x),                              \
        .invert_y = DT_INST_PROP(n, invert_y),                              \
    };                                                                        \
                                                                              \
    PM_DEVICE_DT_INST_DEFINE(n, adns5050_pm_action);                        \
                                                                              \
    DEVICE_DT_INST_DEFINE(n, adns5050_init, PM_DEVICE_DT_INST_GET(n),      \
                          &adns5050_data_##n, &adns5050_config_##n,         \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ADNS5050_INIT)

/*
 * ADNS5050 デバッグテストプログラム
 * このコードをadns5050.cの最後に追加してテスト
 */

#include <zephyr/shell/shell.h>

static int cmd_adns5050_test(const struct shell *shell, size_t argc, char **argv)
{
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(trackball));
    int ret;
    uint8_t val;
    
    if (!device_is_ready(dev)) {
        shell_error(shell, "ADNS5050 device not ready");
        return -ENODEV;
    }
    
    shell_print(shell, "=== ADNS5050 Debug Test ===");
    
    /* Product ID読み取りテスト */
    ret = adns5050_read_reg(dev, ADNS5050_REG_PRODUCT_ID, &val);
    if (ret < 0) {
        shell_error(shell, "Failed to read Product ID: %d", ret);
        return ret;
    }
    shell_print(shell, "Product ID: 0x%02x (expected 0x12)", val);
    
    /* Product ID2読み取りテスト */
    ret = adns5050_read_reg(dev, ADNS5050_REG_PRODUCT_ID2, &val);
    if (ret < 0) {
        shell_error(shell, "Failed to read Product ID2: %d", ret);
        return ret;
    }
    shell_print(shell, "Product ID2: 0x%02x (expected 0x26)", val);
    
    /* Revision ID読み取りテスト */
    ret = adns5050_read_reg(dev, ADNS5050_REG_REVISION_ID, &val);
    if (ret < 0) {
        shell_error(shell, "Failed to read Revision ID: %d", ret);
        return ret;
    }
    shell_print(shell, "Revision ID: 0x%02x", val);
    
    /* Motion Status読み取りテスト */
    ret = adns5050_read_reg(dev, ADNS5050_REG_MOTION, &val);
    if (ret < 0) {
        shell_error(shell, "Failed to read Motion register: %d", ret);
        return ret;
    }
    shell_print(shell, "Motion register: 0x%02x (bit 7 set = motion detected)", val);
    
    /* SQUALテスト */
    ret = adns5050_read_reg(dev, ADNS5050_REG_SQUAL, &val);
    if (ret < 0) {
        shell_error(shell, "Failed to read SQUAL: %d", ret);
        return ret;
    }
    shell_print(shell, "SQUAL (surface quality): %d/128", val);
    
    return 0;
}

static int cmd_adns5050_motion(const struct shell *shell, size_t argc, char **argv)
{
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(trackball));
    int ret;
    int16_t x, y;
    int count = 10;
    
    if (argc > 1) {
        count = atoi(argv[1]);
    }
    
    shell_print(shell, "Reading motion data %d times...", count);
    
    for (int i = 0; i < count; i++) {
        ret = adns5050_motion_burst_read(dev, &x, &y);
        if (ret < 0) {
            shell_error(shell, "Failed to read motion: %d", ret);
            return ret;
        }
        
        if (x != 0 || y != 0) {
            shell_print(shell, "[%d] X: %d, Y: %d", i, x, y);
        }
        
        k_sleep(K_MSEC(100));
    }
    
    return 0;
}

static int cmd_adns5050_spi_test(const struct shell *shell, size_t argc, char **argv)
{
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(trackball));
    const struct adns5050_config *cfg = dev->config;
    int ret;
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];
    
    shell_print(shell, "=== SPI Communication Test ===");
    
    /* 基本的なSPI通信テスト */
    tx_buf[0] = 0x00; /* Product ID register (read) */
    tx_buf[1] = 0x00;
    
    const struct spi_buf tx_bufs[] = {
        {.buf = tx_buf, .len = 2}
    };
    const struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count = 1
    };
    
    const struct spi_buf rx_bufs[] = {
        {.buf = rx_buf, .len = 2}
    };
    const struct spi_buf_set rx = {
        .buffers = rx_bufs,
        .count = 1
    };
    
    shell_print(shell, "Sending: 0x%02x 0x%02x", tx_buf[0], tx_buf[1]);
    
    ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
    if (ret < 0) {
        shell_error(shell, "SPI transceive failed: %d", ret);
        return ret;
    }
    
    shell_print(shell, "Received: 0x%02x 0x%02x", rx_buf[0], rx_buf[1]);
    
    return 0;
}

static int cmd_adns5050_reset(const struct shell *shell, size_t argc, char **argv)
{
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(trackball));
    int ret;
    
    shell_print(shell, "Resetting ADNS5050...");
    
    ret = adns5050_write_reg(dev, ADNS5050_REG_CHIP_RESET, ADNS5050_RESET_VALUE);
    if (ret < 0) {
        shell_error(shell, "Reset failed: %d", ret);
        return ret;
    }
    
    k_sleep(K_MSEC(55));
    shell_print(shell, "Reset complete");
    
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(adns5050_cmds,
    SHELL_CMD(test, NULL, "Basic ADNS5050 test", cmd_adns5050_test),
    SHELL_CMD(motion, NULL, "Read motion data [count]", cmd_adns5050_motion),
    SHELL_CMD(spi, NULL, "Test SPI communication", cmd_adns5050_spi_test),
    SHELL_CMD(reset, NULL, "Reset ADNS5050", cmd_adns5050_reset),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(adns5050, &adns5050_cmds, "ADNS5050 debug commands", NULL);