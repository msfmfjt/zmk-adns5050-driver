ADNS5050 driver implementation for ZMK with at least Zephyr 3.5

This work is based on [ufan's zmk pixart sensor drivers](https://github.com/ufan/zmk/tree/support-trackpad) and [inorichi's zmk-pmw3610-driver](https://github.com/inorichi/zmk-pmw3610-driver).

**Important:** This driver now uses **GPIO bit-banging** instead of SPI, matching QMK's implementation for better hardware compatibility.

## Features

- **GPIO bit-banging communication** (compatible with QMK hardware setups)
- **Polling-based operation** (no IRQ pin required)
- Configurable polling interval (default 8ms for 125Hz)
- Support for scroll layers and snipe layers
- Configurable sensor orientation
- X/Y axis inversion support
- Auto-mouse layer activation
- Direct GPIO control for SCLK, SDIO (bidirectional), and CS pins

## Installation

Only GitHub actions builds are covered here. Local builds are different for each user, therefore it's not possible to cover all cases.

Include this project on your ZMK's west manifest in `config/west.yml`:

```yml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/petejohanson
    - name: msfmfjt
      url-base: https://github.com/msfmfjt
  projects:
    - name: zmk
      remote: zmkfirmware
      revision: feat/pointers-move-scroll
      import: app/west.yml
    - name: zmk-adns5050-driver
      remote: msfmfjt
      revision: main
  self:
    path: config
```

Then, edit your `build.yml` to look like this, 3.5 is now on main:

```yml
on: [workflow_dispatch]

jobs:
  build:
    uses: zmkfirmware/zmk/.github/workflows/build-user-config.yml@main
```

Now, update your `board.overlay` adding the necessary bits (update the pins for your board accordingly):

```dts
/ {
    trackball: trackball {
        status = "okay";
        compatible = "pixart,adns5050";
        
        // GPIO pin assignments (update for your board)
        sclk-gpios = <&gpio0 8 GPIO_ACTIVE_HIGH>;    // Serial clock pin
        sdio-gpios = <&gpio0 17 GPIO_ACTIVE_HIGH>;   // Serial data I/O pin (bidirectional)
        cs-gpios = <&gpio0 20 GPIO_ACTIVE_LOW>;      // Chip select pin

        /*   optional features   */
        // snipe-layers = <1>;
        // scroll-layers = <2 3>;
        // automouse-layer = <4>;
    };

    trackball_listener {
        compatible = "zmk,input-listener";
        device = <&trackball>;
    };
};
```

## Pin Configuration

The ADNS5050 requires three GPIO pins:

- **SCLK (Serial Clock)**: Connect to your chosen GPIO pin
- **SDIO (Serial Data I/O)**: Bidirectional data pin, connect to your chosen GPIO pin  
- **CS (Chip Select)**: Active-low chip select pin

**Important:** The SDIO pin must support both input and output modes as it's bidirectional. The driver automatically switches between input (during reads) and output (during writes) modes.

Now enable the driver config in your `board.config` file (read the Kconfig file to find out all possible options):

```conf
# Core requirements for GPIO-based ADNS5050 driver
CONFIG_GPIO=y
CONFIG_INPUT=y
CONFIG_ZMK_MOUSE=y
CONFIG_ADNS5050=y

# Optional: Set polling interval (default 8ms = 125Hz)
# CONFIG_ADNS5050_POLLING_INTERVAL_MS=8

# Optional: Enable detailed logging for debugging
# CONFIG_LOG=y
# CONFIG_ADNS5050_LOG_LEVEL_DBG=y
```

## Hardware Compatibility

This implementation uses GPIO bit-banging that matches QMK's ADNS5050 driver behavior. If your hardware works with QMK's ADNS5050 driver, it should work with this ZMK implementation using the same pin connections.

## Troubleshooting

- **Product ID read failures**: Ensure GPIO pins are correctly configured and connected
- **No sensor response**: Verify SDIO pin supports bidirectional operation  
- **Timing issues**: The driver uses microsecond-level delays matching QMK's implementation
- **Enable debug logging** to see detailed communication with the sensor
