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
#include <zephyr/drivers/gpio.h>
#include <zmk/keymap.h>
#include "adns5050.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adns5050, CONFIG_ADNS5050_LOG_LEVEL);

// GPIO bit-banging control functions (based on QMK implementation)
static void adns5050_cs_select(const struct device *dev);
static void adns5050_cs_deselect(const struct device *dev);
static void adns5050_serial_write(const struct device *dev, uint8_t data);
static uint8_t adns5050_serial_read(const struct device *dev);

// GPIO bit-banging implementation
static void adns5050_cs_select(const struct device *dev) {
    const struct pixart_config *config = dev->config;
    gpio_pin_set_dt(&config->cs_gpio, 0);  // CS active low
    k_busy_wait(2); // CS setup time (critical for ADNS5050)
    printk("ADNS5050: CS selected (LOW)\n");
}

static void adns5050_cs_deselect(const struct device *dev) {
    const struct pixart_config *config = dev->config;
    k_busy_wait(2); // CS hold time before release
    gpio_pin_set_dt(&config->cs_gpio, 1);  // CS inactive high
    k_busy_wait(2); // CS recovery time
    printk("ADNS5050: CS deselected (HIGH)\n");
}

static void adns5050_serial_write(const struct device *dev, uint8_t data) {
    const struct pixart_config *config = dev->config;
    
    // Configure SDIO as output for writing
    gpio_pin_configure_dt(&config->sdio_gpio, GPIO_OUTPUT);
    k_busy_wait(2); // Allow pin configuration to settle
    
    printk("ADNS5050: Writing 0x%02x - ", data);
    
    // Send 8 bits, MSB first
    for (int i = 7; i >= 0; i--) {
        int bit_val = (data >> i) & 1;
        
        // Set clock low and setup data
        gpio_pin_set_dt(&config->sclk_gpio, 0);
        gpio_pin_set_dt(&config->sdio_gpio, bit_val);
        k_busy_wait(2); // Setup time
        
        // Clock high to latch data
        gpio_pin_set_dt(&config->sclk_gpio, 1);
        k_busy_wait(2); // Hold time
        
        printk("%d", bit_val); // Debug: print each bit
    }
    
    // Leave clock low after transmission
    gpio_pin_set_dt(&config->sclk_gpio, 0);
    k_busy_wait(2);
    
    printk("\n");
}

static uint8_t adns5050_serial_read(const struct device *dev) {
    const struct pixart_config *config = dev->config;
    uint8_t data = 0;
    
    // Configure SDIO as input with pull-up for reading (critical for ADNS5050)
    gpio_pin_configure_dt(&config->sdio_gpio, GPIO_INPUT | GPIO_PULL_UP);
    k_busy_wait(2); // Allow pin configuration to settle
    
    printk("ADNS5050: Starting serial read - ");
    
    // Read 8 bits, MSB first
    for (int i = 7; i >= 0; i--) {
        // Set clock low first and wait
        gpio_pin_set_dt(&config->sclk_gpio, 0);
        k_busy_wait(2); // Increased delay for stability
        
        // Set clock high to trigger data
        gpio_pin_set_dt(&config->sclk_gpio, 1);
        k_busy_wait(1); // Setup time
        
        // Read data bit after clock edge
        int bit_val = gpio_pin_get_dt(&config->sdio_gpio);
        if (bit_val) {
            data |= (1 << i);
        }
        printk("%d", bit_val); // Debug: print each bit
        
        k_busy_wait(2); // Hold time
    }
    
    // Leave clock low after transmission
    gpio_pin_set_dt(&config->sclk_gpio, 0);
    k_busy_wait(2);
    
    printk(" = 0x%02x\n", data);
    return data;
}

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

// GPIO-based register read function
static int reg_read(const struct device *dev, uint8_t reg, uint8_t *buf) {
    printk("ADNS5050: GPIO reg_read - register 0x%02x\\n", reg);
    
    // Select chip
    adns5050_cs_select(dev);
    k_busy_wait(1); // 1μs delay after CS select
    
    // Send register address (read command)
    adns5050_serial_write(dev, reg);
    k_busy_wait(160); // 160μs delay (increased for better stability)
    
    // Read data
    *buf = adns5050_serial_read(dev);
    
    // Deselect chip
    adns5050_cs_deselect(dev);
    k_busy_wait(20); // 20μs delay (t_SRX from QMK)
    
    printk("ADNS5050: GPIO reg_read - register 0x%02x = 0x%02x\\n", reg, *buf);
    return 0;
}

// GPIO-based register write function
static int reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    printk("ADNS5050: GPIO reg_write - register 0x%02x = 0x%02x\\n", reg, val);
    
    // Select chip
    adns5050_cs_select(dev);
    k_busy_wait(1); // 1μs delay after CS select
    
    // Send register address with write bit (0x80)
    adns5050_serial_write(dev, reg | 0x80);
    
    // Send data
    adns5050_serial_write(dev, val);
    k_busy_wait(35); // 35μs delay (t_SCLK_NCS_WR from QMK)
    
    // Deselect chip
    adns5050_cs_deselect(dev);
    k_busy_wait(180); // 180μs delay (t_SWX from QMK)
    
    return 0;
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
    
    // Extended stabilization time after reset
    k_msleep(200);
    
    // Wake up device with multiple dummy reads
    uint8_t dummy;
    for (int i = 0; i < 3; i++) {
        reg_read(dev, ADNS5050_REG_MOTION, &dummy);
        k_msleep(10);
    }
    
    // Try multiple reads with increasing delays
    for (int attempt = 0; attempt < 5; attempt++) {
        printk("ADNS5050: Read attempt %d\n", attempt + 1);
        
        // Additional wake-up read before product ID read
        reg_read(dev, ADNS5050_REG_MOTION, &dummy);
        k_msleep(20);
        
        // Try reading multiple registers for diagnosis
        uint8_t revision_id = 0;
        int err1 = reg_read(dev, ADNS5050_REG_REVISION_ID, &revision_id);
        k_msleep(10);
        
        int err2 = reg_read(dev, ADNS5050_REG_PRODUCT_ID, &product_id);
        
        printk("ADNS5050: Attempt %d - Read results:\n", attempt + 1);
        printk("  Revision ID: 0x%02x (err: %d)\n", revision_id, err1);
        printk("  Product ID: 0x%02x (err: %d, expected: 0x%02x)\n", product_id, err2, ADNS5050_PRODUCT_ID);
        
        if (err2) {
            printk("ADNS5050: GPIO read error: %d\n", err2);
            k_msleep(100);
            continue;
        }
               
        if (product_id == ADNS5050_PRODUCT_ID) {
            printk("ADNS5050: Product ID verification successful!\n");
            return 0;
        }
        
        // Progressive delay increase
        k_msleep(50 + (attempt * 25));
    }

    LOG_ERR("Incorrect product id 0x%x (expecting 0x%x)!", product_id, ADNS5050_PRODUCT_ID);
    return -EIO;
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

    /* Reset GPIO state and ensure clean startup */
    adns5050_cs_deselect(dev);
    k_msleep(10); // Increased delay for power stabilization
    adns5050_cs_select(dev);
    k_msleep(10);
    adns5050_cs_deselect(dev);
    k_msleep(10);

    /* Wake up device with dummy read before reset */
    uint8_t dummy;
    reg_read(dev, ADNS5050_REG_PRODUCT_ID, &dummy);
    k_msleep(50);

    /* Reset the ADNS5050 chip */
    return reg_write(dev, ADNS5050_REG_CHIP_RESET, ADNS5050_RESET_CMD);
}

// ADNS5050 doesn't need observation register clearing

static int adns5050_async_init_configure(const struct device *dev) {
    LOG_INF("ADNS5050 async_init_configure");
    const struct pixart_config *config = dev->config;

    int err = 0;

    // Test CS GPIO functionality with readback
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
    
    // Test SCLK GPIO
    printk("ADNS5050: Testing SCLK GPIO...\n");
    gpio_pin_set_dt(&config->sclk_gpio, 0);
    k_msleep(1);
    gpio_pin_set_dt(&config->sclk_gpio, 1);
    k_msleep(1);
    gpio_pin_set_dt(&config->sclk_gpio, 0);
    printk("ADNS5050: SCLK GPIO test completed\n");
    
    // Test SDIO GPIO
    printk("ADNS5050: Testing SDIO GPIO...\n");
    gpio_pin_configure_dt(&config->sdio_gpio, GPIO_OUTPUT);
    gpio_pin_set_dt(&config->sdio_gpio, 0);
    k_msleep(1);
    gpio_pin_set_dt(&config->sdio_gpio, 1);
    k_msleep(1);
    gpio_pin_configure_dt(&config->sdio_gpio, GPIO_INPUT | GPIO_PULL_UP);
    k_msleep(1);
    int sdio_state = gpio_pin_get_dt(&config->sdio_gpio);
    printk("ADNS5050: SDIO GPIO test completed (pulled-up state: %d)\n", sdio_state);

    // Check GPIO readiness
    if (!device_is_ready(config->sclk_gpio.port)) {
        printk("ADNS5050: SCLK GPIO device is NOT ready!\n");
        return -ENODEV;
    }
    if (!device_is_ready(config->sdio_gpio.port)) {
        printk("ADNS5050: SDIO GPIO device is NOT ready!\n");
        return -ENODEV;
    }
    printk("ADNS5050: All GPIO devices are ready\n");

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

    // ADNS5050 doesn't need CPI division, use fixed resolution
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

    printk("ADNS5050: GPIO pins - SCLK: %p:%d, SDIO: %p:%d, CS: %p:%d\n", 
           config->sclk_gpio.port, config->sclk_gpio.pin,
           config->sdio_gpio.port, config->sdio_gpio.pin,
           config->cs_gpio.port, config->cs_gpio.pin);

    // init device pointer
    data->dev = dev;

    // ADNS5050 doesn't have smart algorithm
    data->sw_smart_flag = false;

    // init trigger handler work
    k_work_init(&data->trigger_work, adns5050_work_callback);

    // init polling timer
    k_timer_init(&data->polling_timer, adns5050_polling_callback, NULL);

    // Configure GPIO pins
    if (!device_is_ready(config->cs_gpio.port)) {
        LOG_ERR("CS GPIO device not ready");
        return -ENODEV;
    }
    if (!device_is_ready(config->sclk_gpio.port)) {
        LOG_ERR("SCLK GPIO device not ready");
        return -ENODEV;
    }
    if (!device_is_ready(config->sdio_gpio.port)) {
        LOG_ERR("SDIO GPIO device not ready");
        return -ENODEV;
    }

    // Configure CS pin as output (inactive high)
    err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Cannot configure CS GPIO");
        return err;
    }

    // Configure SCLK pin as output (initially low)
    err = gpio_pin_configure_dt(&config->sclk_gpio, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Cannot configure SCLK GPIO");
        return err;
    }

    // Configure SDIO pin as output initially (will be switched to input during reads)
    err = gpio_pin_configure_dt(&config->sdio_gpio, GPIO_OUTPUT);
    if (err) {
        LOG_ERR("Cannot configure SDIO GPIO");
        return err;
    }

    // Setup delayable and non-blocking init jobs for ADNS5050:
    // 1. power reset
    // 2. configure sensor
    k_work_init_delayable(&data->init_work, adns5050_async_init);

    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));

    return err;
}

// Define the device using the DT node directly (not instance-based)
#define TRACKBALL_NODE DT_NODELABEL(trackball)

#if DT_NODE_EXISTS(TRACKBALL_NODE)
static struct pixart_data trackball_data;
static int32_t trackball_scroll_layers[] = DT_PROP_OR(TRACKBALL_NODE, scroll_layers, {});
static int32_t trackball_snipe_layers[] = DT_PROP_OR(TRACKBALL_NODE, snipe_layers, {});

static const struct pixart_config trackball_config = {
    .sclk_gpio = GPIO_DT_SPEC_GET(TRACKBALL_NODE, sclk_gpios),
    .sdio_gpio = GPIO_DT_SPEC_GET(TRACKBALL_NODE, sdio_gpios),
    .cs_gpio = GPIO_DT_SPEC_GET(TRACKBALL_NODE, cs_gpios),
    .scroll_layers = trackball_scroll_layers,
    .scroll_layers_len = DT_PROP_LEN_OR(TRACKBALL_NODE, scroll_layers, 0),
    .snipe_layers = trackball_snipe_layers,
    .snipe_layers_len = DT_PROP_LEN_OR(TRACKBALL_NODE, snipe_layers, 0),
};

DEVICE_DT_DEFINE(TRACKBALL_NODE, adns5050_init, NULL, &trackball_data, &trackball_config, 
                 POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, NULL);
#endif
