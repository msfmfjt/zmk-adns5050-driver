/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_adns5050

// 12-bit two's complement value to int16_t
// adapted from https://stackoverflow.com/questions/70802306/convert-a-12-bit-signed-number-in-c
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zmk/keymap.h>
#include "adns5050.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adns5050, CONFIG_ADNS5050_LOG_LEVEL);

//////// Sensor initialization steps definition //////////
// init is done in non-blocking manner (i.e., async), a //
// delayable work is defined for this purpose           //
enum adns5050_init_step {
    ASYNC_INIT_STEP_POWER_UP,  // reset sensor and power-up
    ASYNC_INIT_STEP_CONFIGURE, // configure sensor settings
    ASYNC_INIT_STEP_COUNT // end flag
};

/* Timings (in ms) needed in between steps to allow each step finishes successfully. */
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 50,   // ADNS5050 power-up time
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int adns5050_async_init_power_up(const struct device *dev);
static int adns5050_async_init_configure(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = adns5050_async_init_power_up,
    [ASYNC_INIT_STEP_CONFIGURE] = adns5050_async_init_configure,
};

//////// Function definitions //////////

// checked and keep
static int spi_cs_ctrl(const struct device *dev, bool enable) {
    const struct pixart_config *config = dev->config;
    int err;

    if (!enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    err = gpio_pin_set_dt(&config->cs_gpio, (int)enable);
    if (err) {
        LOG_ERR("SPI CS ctrl failed");
    }

    if (enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    return err;
}

// checked and keep
static int reg_read(const struct device *dev, uint8_t reg, uint8_t *buf) {
    int err;
    /* struct pixart_data *data = dev->data; */
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    /* Write register address. */
    const struct spi_buf tx_buf = {.buf = &reg, .len = 1};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg read failed on SPI write");
        return err;
    }

    k_busy_wait(T_SRAD);

    /* Read register value. */
    struct spi_buf rx_buf = {
        .buf = buf,
        .len = 1,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Reg read failed on SPI read");
        return err;
    }

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_SRX);

    return 0;
}

// primitive write without enable/disable spi clock on the sensor
static int _reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    int err;
    /* struct pixart_data *data = dev->data; */
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    uint8_t buf[] = {SPI_WRITE_BIT | reg, val};
    const struct spi_buf tx_buf = {.buf = buf, .len = ARRAY_SIZE(buf)};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg write failed on SPI write");
        return err;
    }

    k_busy_wait(T_SCLK_NCS_WR);

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_SWX);

    return 0;
}

static int reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    // ADNS5050 doesn't need SPI clock management
    return _reg_write(dev, reg, val);
}

static int motion_read(const struct device *dev, int16_t *delta_x, int16_t *delta_y) {
    int err;
    uint8_t motion_reg;
    uint8_t delta_x_reg;
    uint8_t delta_y_reg;

    // Read motion register first to check if motion occurred
    err = reg_read(dev, ADNS5050_REG_MOTION, &motion_reg);
    if (err) {
        return err;
    }

    if (!(motion_reg & 0x80)) {
        // No motion detected
        *delta_x = 0;
        *delta_y = 0;
        return 0;
    }

    // Read delta X
    err = reg_read(dev, ADNS5050_REG_DELTA_X, &delta_x_reg);
    if (err) {
        return err;
    }

    // Read delta Y
    err = reg_read(dev, ADNS5050_REG_DELTA_Y, &delta_y_reg);
    if (err) {
        return err;
    }

    // Convert to signed 8-bit values
    *delta_x = (int8_t)delta_x_reg;
    *delta_y = (int8_t)delta_y_reg;

    return 0;
}

// ADNS5050 doesn't support motion burst read functionality
// This function is kept for potential compatibility but always returns error
static int motion_burst_read_legacy(const struct device *dev, uint8_t *buf, size_t burst_size) {
    LOG_WRN("Motion burst read not supported on ADNS5050");
    return -ENOTSUP;
}

// ADNS5050 doesn't need burst write functionality
// Single register writes are sufficient

static int check_product_id(const struct device *dev) {
    uint8_t product_id = 0x01;
    
    printk("ADNS5050: Attempting to read product ID...\n");
    int err = reg_read(dev, ADNS5050_REG_PRODUCT_ID, &product_id);
    if (err) {
        printk("ADNS5050: SPI read error: %d\n", err);
        LOG_ERR("Cannot obtain product id");
        return err;
    }

    printk("ADNS5050: Read product ID: 0x%02x (expected: 0x%02x)\n", product_id, ADNS5050_PRODUCT_ID);
    
    if (product_id != ADNS5050_PRODUCT_ID) {
        LOG_ERR("Incorrect product id 0x%x (expecting 0x%x)!", product_id, ADNS5050_PRODUCT_ID);
        return -EIO;
    }

    printk("ADNS5050: Product ID verification successful!\n");
    return 0;
}

// ADNS5050 has a fixed resolution and doesn't support CPI configuration
// This function is kept for compatibility but does nothing
static int set_cpi(const struct device *dev, uint32_t cpi) {
    struct pixart_data *dev_data = dev->data;
    dev_data->curr_cpi = 400; // ADNS5050 typical resolution
    LOG_INF("ADNS5050 uses fixed resolution (CPI setting ignored)");
    return 0;
}

static int set_cpi_if_needed(const struct device *dev, uint32_t cpi) {
    struct pixart_data *data = dev->data;
    if (cpi != data->curr_cpi) {
        return set_cpi(dev, cpi);
    }
    return 0;
}

// ADNS5050 doesn't support configurable sample time
static int set_sample_time(const struct device *dev, uint8_t reg_addr, uint32_t sample_time) {
    LOG_INF("ADNS5050 uses fixed sample time (setting ignored)");
    return 0;
}

// ADNS5050 doesn't support configurable downshift time
static int set_downshift_time(const struct device *dev, uint8_t reg_addr, uint32_t time) {
    LOG_INF("ADNS5050 doesn't support downshift time configuration");
    return 0;
}


static int adns5050_async_init_power_up(const struct device *dev) {
    LOG_INF("ADNS5050 async_init_power_up");

    /* Reset spi port */
    spi_cs_ctrl(dev, false);
    spi_cs_ctrl(dev, true);

    /* Reset the ADNS5050 chip */
    return reg_write(dev, ADNS5050_REG_CHIP_RESET, ADNS5050_RESET_CMD);
}

// ADNS5050 doesn't need observation register clearing

static int adns5050_async_init_configure(const struct device *dev) {
    LOG_INF("ADNS5050 async_init_configure");
    const struct pixart_config *config = dev->config;

    int err = 0;

    // Test CS GPIO functionality
    printk("ADNS5050: Testing CS GPIO control...\n");
    err = gpio_pin_set_dt(&config->cs_gpio, 1);
    if (err) {
        printk("ADNS5050: CS GPIO set HIGH failed: %d\n", err);
    } else {
        printk("ADNS5050: CS GPIO set HIGH successful\n");
    }
    
    k_msleep(1);
    
    err = gpio_pin_set_dt(&config->cs_gpio, 0);
    if (err) {
        printk("ADNS5050: CS GPIO set LOW failed: %d\n", err);
    } else {
        printk("ADNS5050: CS GPIO set LOW successful\n");
    }

    // Check SPI bus readiness
    if (!device_is_ready(config->bus.bus)) {
        printk("ADNS5050: SPI bus device is NOT ready!\n");
        return -ENODEV;
    } else {
        printk("ADNS5050: SPI bus device is ready\n");
    }

    // Check product ID
    err = check_product_id(dev);
    if (err) {
        LOG_ERR("Failed checking product id");
        return err;
    }

    // Clear motion registers by reading them
    uint8_t buf[1];
    err = reg_read(dev, ADNS5050_REG_MOTION, buf);
    if (!err) {
        err = reg_read(dev, ADNS5050_REG_DELTA_X, buf);
    }
    if (!err) {
        err = reg_read(dev, ADNS5050_REG_DELTA_Y, buf);
    }

    // Set up default CPI (ADNS5050 doesn't support CPI configuration but we track it)
    if (!err) {
        err = set_cpi(dev, 400); // Default ADNS5050 resolution
    }

    if (err) {
        LOG_ERR("Config the ADNS5050 sensor failed");
        return err;
    }

    LOG_INF("ADNS5050 configuration completed");
    return 0;
}

static void adns5050_async_init(struct k_work *work) {
    struct k_work_delayable *work2 = (struct k_work_delayable *)work;
    struct pixart_data *data = CONTAINER_OF(work2, struct pixart_data, init_work);
    const struct device *dev = data->dev;

    LOG_INF("ADNS5050 async init step %d", data->async_init_step);

    data->err = async_init_fn[data->async_init_step](dev);
    if (data->err) {
        LOG_ERR("ADNS5050 initialization failed");
    } else {
        data->async_init_step++;

        if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
            data->ready = true; // sensor is ready to work
            LOG_INF("ADNS5050 initialized");
            // Start polling timer
            k_timer_start(&data->polling_timer, K_MSEC(CONFIG_ADNS5050_POLLING_INTERVAL_MS), 
                         K_MSEC(CONFIG_ADNS5050_POLLING_INTERVAL_MS));
        } else {
            k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
        }
    }
}

#define AUTOMOUSE_LAYER (DT_PROP(DT_DRV_INST(0), automouse_layer))
#if AUTOMOUSE_LAYER > 0
struct k_timer automouse_layer_timer;
static bool automouse_triggered = false;

static void activate_automouse_layer() {
    automouse_triggered = true;
    zmk_keymap_layer_activate(AUTOMOUSE_LAYER);
    k_timer_start(&automouse_layer_timer, K_MSEC(CONFIG_ADNS5050_AUTOMOUSE_TIMEOUT_MS), K_NO_WAIT);
}

static void deactivate_automouse_layer(struct k_timer *timer) {
    automouse_triggered = false;
    zmk_keymap_layer_deactivate(AUTOMOUSE_LAYER);
}

K_TIMER_DEFINE(automouse_layer_timer, deactivate_automouse_layer, NULL);
#endif

static enum pixart_input_mode get_input_mode_for_current_layer(const struct device *dev) {
    const struct pixart_config *config = dev->config;
    uint8_t curr_layer = zmk_keymap_highest_layer_active();
    for (size_t i = 0; i < config->scroll_layers_len; i++) {
        if (curr_layer == config->scroll_layers[i]) {
            return SCROLL;
        }
    }
    for (size_t i = 0; i < config->snipe_layers_len; i++) {
        if (curr_layer == config->snipe_layers[i]) {
            return SNIPE;
        }
    }
    return MOVE;
}

static int adns5050_report_data(const struct device *dev) {
    struct pixart_data *data = dev->data;

    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    int32_t dividor = 1; // ADNS5050 doesn't need CPI division
    enum pixart_input_mode input_mode = get_input_mode_for_current_layer(dev);
    bool input_mode_changed = data->curr_mode != input_mode;
    
    if (input_mode == SCROLL && input_mode_changed) {
        data->scroll_delta_x = 0;
        data->scroll_delta_y = 0;
    }

    data->curr_mode = input_mode;

#if AUTOMOUSE_LAYER > 0
    if (input_mode == MOVE &&
            (automouse_triggered || zmk_keymap_highest_layer_active() != AUTOMOUSE_LAYER)
    ) {
        activate_automouse_layer();
    }
#endif

    int16_t raw_x, raw_y;
    int err = motion_read(dev, &raw_x, &raw_y);
    if (err) {
        return err;
    }

    int16_t x = raw_x;
    int16_t y = raw_y;

    // Apply orientation transformation
    if (IS_ENABLED(CONFIG_ADNS5050_ORIENTATION_0)) {
        x = -raw_x;
        y = raw_y;
    } else if (IS_ENABLED(CONFIG_ADNS5050_ORIENTATION_90)) {
        x = raw_y;
        y = -raw_x;
    } else if (IS_ENABLED(CONFIG_ADNS5050_ORIENTATION_180)) {
        x = raw_x;
        y = -raw_y;
    } else if (IS_ENABLED(CONFIG_ADNS5050_ORIENTATION_270)) {
        x = -raw_y;
        y = raw_x;
    }

    if (IS_ENABLED(CONFIG_ADNS5050_INVERT_X)) {
        x = -x;
    }

    if (IS_ENABLED(CONFIG_ADNS5050_INVERT_Y)) {
        y = -y;
    }

    if (x != 0 || y != 0) {
        if (input_mode != SCROLL) {
            input_report_rel(dev, INPUT_REL_X, x, false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, y, true, K_FOREVER);
        } else {
            data->scroll_delta_x += x;
            data->scroll_delta_y += y;
            if (abs(data->scroll_delta_y) > CONFIG_ADNS5050_SCROLL_TICK) {
                input_report_rel(dev, INPUT_REL_WHEEL,
                                 data->scroll_delta_y > 0 ? ADNS5050_SCROLL_Y_NEGATIVE : ADNS5050_SCROLL_Y_POSITIVE,
                                 true, K_FOREVER);
                data->scroll_delta_x = 0;
                data->scroll_delta_y = 0;
            } else if (abs(data->scroll_delta_x) > CONFIG_ADNS5050_SCROLL_TICK) {
                input_report_rel(dev, INPUT_REL_HWHEEL,
                                 data->scroll_delta_x > 0 ? ADNS5050_SCROLL_X_NEGATIVE : ADNS5050_SCROLL_X_POSITIVE,
                                 true, K_FOREVER);
                data->scroll_delta_x = 0;
                data->scroll_delta_y = 0;
            }
        }
    }

    return err;
}


static void adns5050_work_callback(struct k_work *work) {
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);
    const struct device *dev = data->dev;

    adns5050_report_data(dev);
}

static void adns5050_polling_callback(struct k_timer *timer) {
    struct pixart_data *data = CONTAINER_OF(timer, struct pixart_data, polling_timer);
    
    // Submit work to process motion data
    k_work_submit(&data->trigger_work);
}


static int adns5050_init(const struct device *dev) {
    printk("ADNS5050: Driver init function called for device %s!\n", dev->name);
    LOG_INF("Start initializing ADNS5050...");

    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err;

    printk("ADNS5050: SPI bus frequency: %d Hz\n", config->bus.config.frequency);
    printk("ADNS5050: CS GPIO port: %p, pin: %d\n", config->cs_gpio.port, config->cs_gpio.pin);

    // init device pointer
    data->dev = dev;

    // ADNS5050 doesn't have smart algorithm
    data->sw_smart_flag = false;

    // init trigger handler work
    k_work_init(&data->trigger_work, adns5050_work_callback);

    // init polling timer
    k_timer_init(&data->polling_timer, adns5050_polling_callback, NULL);

    // check readiness of cs gpio pin and init it to inactive
    if (!device_is_ready(config->cs_gpio.port)) {
        LOG_ERR("SPI CS device not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Cannot configure SPI CS GPIO");
        return err;
    }

    // Setup delayable and non-blocking init jobs for ADNS5050:
    // 1. power reset
    // 2. configure sensor
    k_work_init_delayable(&data->init_work, adns5050_async_init);

    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));

    return err;
}

#define ADNS5050_DEFINE(n)                                                                          \
    static struct pixart_data data##n;                                                             \
    static int32_t scroll_layers##n[] = DT_PROP_OR(DT_DRV_INST(n), scroll_layers, {});             \
    static int32_t snipe_layers##n[] = DT_PROP_OR(DT_DRV_INST(n), snipe_layers, {});               \
    static const struct pixart_config config##n = {                                                \
        .bus =                                                                                     \
            {                                                                                      \
                .bus = DEVICE_DT_GET(DT_INST_BUS(n)),                                              \
                .config =                                                                          \
                    {                                                                              \
                        .frequency = DT_INST_PROP(n, spi_max_frequency),                           \
                        .operation =                                                               \
                            SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA,    \
                        .slave = DT_INST_REG_ADDR(n),                                              \
                    },                                                                             \
            },                                                                                     \
        .cs_gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_DRV_INST(n)),                                       \
        .scroll_layers = scroll_layers##n,                                                         \
        .scroll_layers_len = DT_PROP_LEN_OR(DT_DRV_INST(n), scroll_layers, 0),                     \
        .snipe_layers = snipe_layers##n,                                                           \
        .snipe_layers_len = DT_PROP_LEN_OR(DT_DRV_INST(n), snipe_layers, 0),                       \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, adns5050_init, NULL, &data##n, &config##n, POST_KERNEL,                \
                          CONFIG_SENSOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ADNS5050_DEFINE)
