ADNS5050 driver implementation for ZMK with at least Zephyr 3.5

This work is based on [ufan's zmk pixart sensor drivers](https://github.com/ufan/zmk/tree/support-trackpad) and [inorichi's zmk-pmw3610-driver](https://github.com/inorichi/zmk-pmw3610-driver).

## Features

- Simplified implementation optimized for ADNS5050
- **Polling-based operation** (no IRQ pin required)
- Configurable polling interval (default 8ms for 125Hz)
- Support for scroll layers and snipe layers
- Configurable sensor orientation
- X/Y axis inversion support
- Auto-mouse layer activation

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
&pinctrl {
    spi0_default: spi0_default {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 8)>,
                <NRF_PSEL(SPIM_MOSI, 0, 17)>,
                <NRF_PSEL(SPIM_MISO, 0, 17)>;
        };
    };

    spi0_sleep: spi0_sleep {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 8)>,
                <NRF_PSEL(SPIM_MOSI, 0, 17)>,
                <NRF_PSEL(SPIM_MISO, 0, 17)>;
            low-power-enable;
        };
    };
};

&spi0 {
    status = "okay";
    compatible = "nordic,nrf-spim";
    pinctrl-0 = <&spi0_default>;
    pinctrl-1 = <&spi0_sleep>;
    pinctrl-names = "default", "sleep";
    cs-gpios = <&gpio0 20 GPIO_ACTIVE_LOW>;

    trackball: trackball@0 {
        status = "okay";
        compatible = "pixart,adns5050";
        reg = <0>;
        spi-max-frequency = <2000000>;

        /*   optional features   */
        // snipe-layers = <1>;
        // scroll-layers = <2 3>;
        // automouse-layer = <4>;
    };
};

/ {
  trackball_listener {
    compatible = "zmk,input-listener";
    device = <&trackball>;

  };
};
```

Now enable the driver config in your `board.config` file (read the Kconfig file to find out all possible options):

```conf
CONFIG_SPI=y
CONFIG_INPUT=y
CONFIG_ZMK_MOUSE=y
CONFIG_ADNS5050=y
# Optional: Set polling interval (default 8ms = 125Hz)
# CONFIG_ADNS5050_POLLING_INTERVAL_MS=8
```
