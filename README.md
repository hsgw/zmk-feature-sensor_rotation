# Sensor Rotation ZMK Module

Work in progress

A ZMK input processor module that rotates XY sensor input (e.g., trackball or pointing stick) by a configurable angle using integer-based lookup tables. This module extends ZMK firmware's input processing capabilities to adjust sensor orientation without floating-point operations.

## Features

- Rotates XY sensor coordinates by a user-defined angle (0° to 360°) as a input processor for ZMK Firmware.
- Uses precomputed sine/cosine lookup tables for efficient integer arithmetic.
- Configurable via device tree (rotation-angle) or Kconfig (CONFIG_SENSOR_ROTATION_ANGLE).
- Compatible with any ZMK-supported input device emitting INPUT_REL_X and INPUT_REL_Y events.

## Configuration

### Device Tree

Override the rotation angle in your keyboard's .dts file to configure this input processor:

```dts
/ {
    trackball_listener {
        compatible = "zmk,input-listener";
        device = <&trackball>;
        input-processors = <&sensor_rotation>;
    };
};

/ {
    input_processors {
        sensor_rotation: sensor_rotation {
            compatible = "zmk,input-processor-sensor-rotation";
            #input-processor-cells = <0>;
            rotation-angle = <90>;
        };
    };
};

#include <#include "input/processors/sensor_rotation.dtsi">
```
