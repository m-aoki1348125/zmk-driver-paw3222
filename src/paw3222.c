/*
 * Copyright 2024 Google LLC
 * Modifications Copyright 2025 sekigon-gonnoc
 *
 * Original source code:
 * https://github.com/zephyrproject-rtos/zephyr/blob/19c6240b6865bcb28e1d786d4dcadfb3a02067a0/drivers/input/input_paw32xx.c
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/util.h>

#include "../include/paw3222.h"

#ifdef CONFIG_ZMK_LAYERS
#include <zmk/layers.h>
#include <zmk/keymap.h>
#include <zmk/events/layer_state_changed.h>
#endif

LOG_MODULE_REGISTER(paw32xx, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT pixart_paw3222

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define PAW32XX_PRODUCT_ID1 0x00
#define PAW32XX_PRODUCT_ID2 0x01
#define PAW32XX_MOTION 0x02
#define PAW32XX_DELTA_X 0x03
#define PAW32XX_DELTA_Y 0x04
#define PAW32XX_OPERATION_MODE 0x05
#define PAW32XX_CONFIGURATION 0x06
#define PAW32XX_WRITE_PROTECT 0x09
#define PAW32XX_SLEEP1 0x0a
#define PAW32XX_SLEEP2 0x0b
#define PAW32XX_SLEEP3 0x0c
#define PAW32XX_CPI_X 0x0d
#define PAW32XX_CPI_Y 0x0e
#define PAW32XX_DELTA_XY_HI 0x12
#define PAW32XX_MOUSE_OPTION 0x19

#define PRODUCT_ID_PAW32XX 0x30
#define SPI_WRITE BIT(7)

#define MOTION_STATUS_MOTION BIT(7)
#define OPERATION_MODE_SLP_ENH BIT(4)
#define OPERATION_MODE_SLP2_ENH BIT(3)
#define OPERATION_MODE_SLP_MASK (OPERATION_MODE_SLP_ENH | OPERATION_MODE_SLP2_ENH)
#define CONFIGURATION_PD_ENH BIT(3)
#define CONFIGURATION_RESET BIT(7)
#define WRITE_PROTECT_ENABLE 0x00
#define WRITE_PROTECT_DISABLE 0x5a
#define MOUSE_OPTION_MOVX_INV_BIT 3
#define MOUSE_OPTION_MOVY_INV_BIT 4

#define PAW32XX_DATA_SIZE_BITS 8

#define RESET_DELAY_MS 2

#define RES_STEP 38
#define RES_MIN (16 * RES_STEP)
#define RES_MAX (127 * RES_STEP)

struct paw32xx_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec irq_gpio;
    struct gpio_dt_spec power_gpio;
    int16_t res_cpi;
    bool force_awake;
    int automouse_layer;
    int scroll_layers;
    int movement_timeout_ms;
    int movement_threshold;
};

struct paw32xx_data {
    const struct device *dev;
    struct k_work motion_work;
    struct gpio_callback motion_cb;
    struct k_timer motion_timer; // Add timer for delayed motion checking
    struct k_timer automouse_timer; // Timer for auto mouse timeout
    int accumulated_movement;       // Track total movement for threshold
    bool automouse_active;          // Current auto mouse layer state
    int original_layer;             // Layer to return to after timeout
    // Smart algorithm data
    int16_t prev_x, prev_y;         // Previous movement values for filtering
    uint32_t movement_count;        // Total movement counter
    int16_t velocity_x, velocity_y; // Current velocity for acceleration
    uint32_t last_movement_time;    // Timestamp of last movement
};

static inline int32_t sign_extend(uint32_t value, uint8_t index) {
    __ASSERT_NO_MSG(index <= 31);

    uint8_t shift = 31 - index;

    return (int32_t)(value << shift) >> shift;
}

static int paw32xx_read_reg(const struct device *dev, uint8_t addr, uint8_t *value) {
    const struct paw32xx_config *cfg = dev->config;
    int ret;

    const struct spi_buf tx_buf = {
        .buf = &addr,
        .len = sizeof(addr),
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    struct spi_buf rx_buf[] = {
        {
            .buf = NULL,
            .len = sizeof(addr),
        },
        {
            .buf = value,
            .len = 1,
        },
    };
    const struct spi_buf_set rx = {
        .buffers = rx_buf,
        .count = ARRAY_SIZE(rx_buf),
    };

    ret = spi_transceive_dt(&cfg->spi, &tx, &rx);

    return ret;
}

static int paw32xx_write_reg(const struct device *dev, uint8_t addr, uint8_t value) {
    const struct paw32xx_config *cfg = dev->config;

    uint8_t write_buf[] = {addr | SPI_WRITE, value};
    const struct spi_buf tx_buf = {
        .buf = write_buf,
        .len = sizeof(write_buf),
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    return spi_write_dt(&cfg->spi, &tx);
}

static int paw32xx_update_reg(const struct device *dev, uint8_t addr, uint8_t mask, uint8_t value) {
    uint8_t val;
    int ret;

    ret = paw32xx_read_reg(dev, addr, &val);
    if (ret < 0) {
        return ret;
    }

    val = (val & ~mask) | (value & mask);

    ret = paw32xx_write_reg(dev, addr, val);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

static int paw32xx_read_xy(const struct device *dev, int16_t *x, int16_t *y) {
    const struct paw32xx_config *cfg = dev->config;
    int ret;

    uint8_t tx_data[] = {
        PAW32XX_DELTA_X,
        0xff,
        PAW32XX_DELTA_Y,
        0xff,
    };
    uint8_t rx_data[sizeof(tx_data)];

    const struct spi_buf tx_buf = {
        .buf = tx_data,
        .len = sizeof(tx_data),
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    struct spi_buf rx_buf = {
        .buf = rx_data,
        .len = sizeof(rx_data),
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
    if (ret < 0) {
        return ret;
    }

    *x = rx_data[1];
    *y = rx_data[3];

    *x = sign_extend(*x, PAW32XX_DATA_SIZE_BITS - 1);
    *y = sign_extend(*y, PAW32XX_DATA_SIZE_BITS - 1);

    return 0;
}

static void paw32xx_motion_timer_handler(struct k_timer *timer) {
    struct paw32xx_data *data = CONTAINER_OF(timer, struct paw32xx_data, motion_timer);
    k_work_submit(&data->motion_work);
}

static void paw32xx_automouse_timeout_handler(struct k_timer *timer) {
    struct paw32xx_data *data = CONTAINER_OF(timer, struct paw32xx_data, automouse_timer);
    
    if (data->automouse_active) {
        #ifdef CONFIG_ZMK_LAYERS
        LOG_DBG("Auto mouse timeout, returning to layer %d", data->original_layer);
        zmk_layers_to(data->original_layer);
        #endif
        data->automouse_active = false;
        data->accumulated_movement = 0;
    }
}

static void paw32xx_apply_smart_algorithm(struct paw32xx_data *data, int16_t *x, int16_t *y) {
    uint32_t current_time = k_uptime_get_32();
    uint32_t dt = current_time - data->last_movement_time;
    
    // Movement threshold - ignore very small movements (noise reduction)
    #ifdef CONFIG_PAW3222_MOVEMENT_THRESHOLD
    if (CONFIG_PAW3222_MOVEMENT_THRESHOLD > 0) {
        int magnitude = abs(*x) + abs(*y);
        if (magnitude < CONFIG_PAW3222_MOVEMENT_THRESHOLD) {
            *x = 0;
            *y = 0;
            return;
        }
    }
    #endif
    
    // Velocity calculation for acceleration
    if (dt > 0 && dt < 100) { // Valid time delta (less than 100ms)
        data->velocity_x = (*x * 1000) / dt; // pixels per second
        data->velocity_y = (*y * 1000) / dt;
        
        // Apply acceleration curve based on velocity
        int16_t vel_magnitude = abs(data->velocity_x) + abs(data->velocity_y);
        
        if (vel_magnitude > 500) { // High velocity threshold
            // Apply acceleration multiplier
            *x = (*x * 150) / 100; // 1.5x acceleration
            *y = (*y * 150) / 100;
        }
    }
    
    // Simple noise filtering - average with previous movement
    if (data->movement_count > 0) {
        *x = (*x + data->prev_x) / 2;
        *y = (*y + data->prev_y) / 2;
    }
    
    // Update tracking variables
    data->prev_x = *x;
    data->prev_y = *y;
    data->movement_count++;
    data->last_movement_time = current_time;
    
    #ifdef CONFIG_PAW3222_DEBUG
    LOG_DBG("Smart algorithm: raw(%d,%d) -> filtered(%d,%d), vel=%d", 
           data->prev_x, data->prev_y, *x, *y, 
           abs(data->velocity_x) + abs(data->velocity_y));
    #endif
}

static void paw32xx_activate_automouse_layer(struct paw32xx_data *data, const struct paw32xx_config *cfg) {
    if (!data->automouse_active && cfg->automouse_layer >= 0) {
        #ifdef CONFIG_ZMK_LAYERS
        data->original_layer = zmk_layers_get_current();
        zmk_layers_to(cfg->automouse_layer);
        #endif
        data->automouse_active = true;
        
        LOG_DBG("Activated auto mouse layer %d (was layer %d)", 
               cfg->automouse_layer, data->original_layer);
        
        // Start timeout timer if configured
        if (cfg->movement_timeout_ms > 0) {
            k_timer_start(&data->automouse_timer, K_MSEC(cfg->movement_timeout_ms), K_NO_WAIT);
        }
    } else if (data->automouse_active && cfg->movement_timeout_ms > 0) {
        // Reset timeout timer on continued movement
        k_timer_start(&data->automouse_timer, K_MSEC(cfg->movement_timeout_ms), K_NO_WAIT);
    }
}

static void paw32xx_motion_work_handler(struct k_work *work) {
    struct paw32xx_data *data = CONTAINER_OF(work, struct paw32xx_data, motion_work);
    const struct device *dev = data->dev;
    const struct paw32xx_config *cfg = dev->config;
    uint8_t val;
    int16_t x, y;
    int ret;

    ret = paw32xx_read_reg(dev, PAW32XX_MOTION, &val);
    if (ret < 0) {
        return;
    }

    if ((val & MOTION_STATUS_MOTION) == 0x00) {
        // No motion detected, re-enable interrupts and wait for next interrupt
        gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);

        if (gpio_pin_get_dt(&cfg->irq_gpio) == 0) {
            return;
        }
    }

    ret = paw32xx_read_xy(dev, &x, &y);
    if (ret < 0) {
        return;
    }

    LOG_DBG("x=%4d y=%4d", x, y);

#ifdef CONFIG_PAW3222_SMART_ALGORITHM
    // Apply smart algorithm processing
    paw32xx_apply_smart_algorithm(data, &x, &y);
    
    // Skip processing if movement was filtered out
    if (x == 0 && y == 0) {
        gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
        return;
    }
#endif

    // Auto mouse layer logic
    if (cfg->automouse_layer >= 0 && cfg->movement_threshold > 0) {
        int movement_magnitude = abs(x) + abs(y);
        data->accumulated_movement += movement_magnitude;
        
        if (!data->automouse_active && data->accumulated_movement >= cfg->movement_threshold) {
            paw32xx_activate_automouse_layer(data, cfg);
        } else if (data->automouse_active && cfg->movement_timeout_ms > 0) {
            // Reset timeout on continued movement
            k_timer_start(&data->automouse_timer, K_MSEC(cfg->movement_timeout_ms), K_NO_WAIT);
        }
        
        LOG_DBG("Movement: %d, accumulated: %d, threshold: %d, active: %s", 
               movement_magnitude, data->accumulated_movement, cfg->movement_threshold,
               data->automouse_active ? "true" : "false");
    }

    input_report_rel(data->dev, INPUT_REL_X, x, false, K_FOREVER);
    input_report_rel(data->dev, INPUT_REL_Y, y, true, K_FOREVER);

    // Schedule next check after 15ms without using interrupts
    k_timer_start(&data->motion_timer, K_MSEC(15), K_NO_WAIT);
}

static void paw32xx_motion_handler(const struct device *gpio_dev, struct gpio_callback *cb,
                                   uint32_t pins) {
    struct paw32xx_data *data = CONTAINER_OF(cb, struct paw32xx_data, motion_cb);
    const struct device *dev = data->dev;
    const struct paw32xx_config *cfg = dev->config;

    // Disable interrupts while timer is active
    gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_DISABLE);

    // Cancel any pending timer
    k_timer_stop(&data->motion_timer);

    // Process motion
    k_work_submit(&data->motion_work);
}

int paw32xx_set_resolution(const struct device *dev, uint16_t res_cpi) {
    uint8_t val;
    int ret;

    if (!IN_RANGE(res_cpi, RES_MIN, RES_MAX)) {
        LOG_ERR("res_cpi out of range: %d", res_cpi);
        return -EINVAL;
    }

    val = res_cpi / RES_STEP;

    ret = paw32xx_write_reg(dev, PAW32XX_WRITE_PROTECT, WRITE_PROTECT_DISABLE);
    if (ret < 0) {
        return ret;
    }

    ret = paw32xx_write_reg(dev, PAW32XX_CPI_X, val);
    if (ret < 0) {
        return ret;
    }

    ret = paw32xx_write_reg(dev, PAW32XX_CPI_Y, val);
    if (ret < 0) {
        return ret;
    }

    ret = paw32xx_write_reg(dev, PAW32XX_WRITE_PROTECT, WRITE_PROTECT_ENABLE);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

int paw32xx_force_awake(const struct device *dev, bool enable) {
    uint8_t val = enable ? 0 : OPERATION_MODE_SLP_MASK;
    int ret;

    ret = paw32xx_write_reg(dev, PAW32XX_WRITE_PROTECT, WRITE_PROTECT_DISABLE);
    if (ret < 0) {
        return ret;
    }

    ret = paw32xx_update_reg(dev, PAW32XX_OPERATION_MODE, OPERATION_MODE_SLP_MASK, val);
    if (ret < 0) {
        return ret;
    }

    ret = paw32xx_write_reg(dev, PAW32XX_WRITE_PROTECT, WRITE_PROTECT_ENABLE);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

static int paw32xx_configure(const struct device *dev) {
    const struct paw32xx_config *cfg = dev->config;
    uint8_t val;
    int ret;

    ret = paw32xx_read_reg(dev, PAW32XX_PRODUCT_ID1, &val);
    if (ret < 0) {
        return ret;
    }

    if (val != PRODUCT_ID_PAW32XX) {
        LOG_ERR("Invalid product id: %02x", val);
        return -ENOTSUP;
    }

    ret = paw32xx_update_reg(dev, PAW32XX_CONFIGURATION, CONFIGURATION_RESET, CONFIGURATION_RESET);
    if (ret < 0) {
        return ret;
    }

    k_sleep(K_MSEC(RESET_DELAY_MS));

    if (cfg->res_cpi > 0) {
        paw32xx_set_resolution(dev, cfg->res_cpi);
    }

    paw32xx_force_awake(dev, cfg->force_awake);

    return 0;
}

static int paw32xx_init(const struct device *dev) {
    const struct paw32xx_config *cfg = dev->config;
    struct paw32xx_data *data = dev->data;
    int ret;

    if (!spi_is_ready_dt(&cfg->spi)) {
        LOG_ERR("%s is not ready", cfg->spi.bus->name);
        return -ENODEV;
    }

    data->dev = dev;

    k_work_init(&data->motion_work, paw32xx_motion_work_handler);
    // Initialize the timer for delayed motion checks
    k_timer_init(&data->motion_timer, paw32xx_motion_timer_handler, NULL);

    // Initialize auto mouse functionality
    k_timer_init(&data->automouse_timer, paw32xx_automouse_timeout_handler, NULL);
    data->accumulated_movement = 0;
    data->automouse_active = false;
    data->original_layer = 0;

    // Initialize smart algorithm data
    data->prev_x = 0;
    data->prev_y = 0;
    data->movement_count = 0;
    data->velocity_x = 0;
    data->velocity_y = 0;
    data->last_movement_time = k_uptime_get_32();
    
    LOG_DBG("PAW3222 initialized: layer=%d, threshold=%d, timeout=%dms", 
           cfg->automouse_layer, cfg->movement_threshold, cfg->movement_timeout_ms);

#if DT_INST_NODE_HAS_PROP(0, power_gpios)
    // Initialize power GPIO if defined
    if (gpio_is_ready_dt(&cfg->power_gpio)) {
        // Configure as output but start with power OFF
        ret = gpio_pin_configure_dt(&cfg->power_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret != 0) {
            LOG_ERR("Power pin configuration failed: %d", ret);
            return ret;
        }

        // Wait 0.5 seconds before turning on power
        k_sleep(K_MSEC(500));

        // Now turn on power
        ret = gpio_pin_set_dt(&cfg->power_gpio, 1);
        if (ret != 0) {
            LOG_ERR("Power pin set failed: %d", ret);
            return ret;
        }

        // Wait for power stabilization
        k_sleep(K_MSEC(10));
    }
#endif

    if (!gpio_is_ready_dt(&cfg->irq_gpio)) {
        LOG_ERR("%s is not ready", cfg->irq_gpio.port->name);
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("Motion pin configuration failed: %d", ret);
        return ret;
    }

    gpio_init_callback(&data->motion_cb, paw32xx_motion_handler, BIT(cfg->irq_gpio.pin));

    ret = gpio_add_callback_dt(&cfg->irq_gpio, &data->motion_cb);
    if (ret < 0) {
        LOG_ERR("Could not set motion callback: %d", ret);
        return ret;
    }

    ret = paw32xx_configure(dev);
    if (ret != 0) {
        LOG_ERR("Device configuration failed: %d", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        LOG_ERR("Motion interrupt configuration failed: %d", ret);
        return ret;
    }

    ret = pm_device_runtime_enable(dev);
    if (ret < 0) {
        LOG_ERR("Failed to enable runtime power management: %d", ret);
        return ret;
    }

    return 0;
}

#ifdef CONFIG_PM_DEVICE
static int paw32xx_pm_action(const struct device *dev, enum pm_device_action action) {
    const struct paw32xx_config *cfg = dev->config;
    int ret;
    uint8_t val;

    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        val = CONFIGURATION_PD_ENH;
        ret = paw32xx_update_reg(dev, PAW32XX_CONFIGURATION, CONFIGURATION_PD_ENH, val);
        if (ret < 0) {
            return ret;
        }

#if DT_INST_NODE_HAS_PROP(0, power_gpios)
        if (gpio_is_ready_dt(&cfg->power_gpio)) {
            ret = gpio_pin_set_dt(&cfg->power_gpio, 0);
            if (ret < 0) {
                LOG_ERR("Failed to disable power: %d", ret);
                return ret;
            }
        }
#endif
        break;

    case PM_DEVICE_ACTION_RESUME:
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
        if (gpio_is_ready_dt(&cfg->power_gpio)) {
            ret = gpio_pin_set_dt(&cfg->power_gpio, 1);
            if (ret < 0) {
                LOG_ERR("Failed to enable power: %d", ret);
                return ret;
            }
            // Wait for power stabilization
            k_sleep(K_MSEC(10));
        }
#endif

        val = 0;
        ret = paw32xx_update_reg(dev, PAW32XX_CONFIGURATION, CONFIGURATION_PD_ENH, val);
        if (ret < 0) {
            return ret;
        }
        break;

    default:
        return -ENOTSUP;
    }

    return 0;
}
#endif

#define PAW32XX_SPI_MODE                                                                           \
    (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB)

#define PAW32XX_INIT(n)                                                                            \
    BUILD_ASSERT(IN_RANGE(DT_INST_PROP_OR(n, res_cpi, RES_MIN), RES_MIN, RES_MAX),                 \
                 "invalid res-cpi");                                                               \
                                                                                                   \
    static const struct paw32xx_config paw32xx_cfg_##n = {                                         \
        .spi = SPI_DT_SPEC_INST_GET(n, PAW32XX_SPI_MODE, 0),                                       \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                           \
        .power_gpio = GPIO_DT_SPEC_INST_GET_OR(n, power_gpios, {0}),                               \
        .res_cpi = DT_INST_PROP_OR(n, res_cpi, -1),                                                \
        .force_awake = DT_INST_PROP(n, force_awake),                                               \
        .automouse_layer = DT_INST_PROP_OR(n, automouse_layer, -1),                                \
        .scroll_layers = DT_INST_PROP_OR(n, scroll_layers, -1),                                    \
        .movement_timeout_ms = DT_INST_PROP_OR(n, movement_timeout_ms, 600),                       \
        .movement_threshold = DT_INST_PROP_OR(n, movement_threshold, 100),                         \
    };                                                                                             \
                                                                                                   \
    static struct paw32xx_data paw32xx_data_##n;                                                   \
                                                                                                   \
    PM_DEVICE_DT_INST_DEFINE(n, paw32xx_pm_action);                                                \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, paw32xx_init, PM_DEVICE_DT_INST_GET(n), &paw32xx_data_##n,            \
                          &paw32xx_cfg_##n, POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PAW32XX_INIT)

#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
