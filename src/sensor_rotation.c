#define DT_DRV_COMPAT zmk_input_processor_sensor_rotation

#include "drivers/input_processor.h"
#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// Configuration structure
struct sensor_rotation_config {
  int rotation_angle; // Angle of rotation in degrees
};

// Data structure to hold XY state and precomputed trig values
struct sensor_rotation_data {
  int32_t x;       // Accumulated X value
  int32_t y;       // Accumulated Y value
  int16_t sin_val; // Precomputed sin value
  int16_t cos_val; // Precomputed cos value
};

// Lookup table for sin and cos (0° to 90° in 5° steps, scaled by 1000)
static const int16_t sin_table[] = {
    0,   87,  174, 259, 342, 423, 500, 574, 643, 707, // 0° to 45°
    766, 819, 866, 906, 940, 966, 985, 996, 1000      // 50° to 90°
};

static const int16_t cos_table[] = {
    1000, 996, 985, 966, 940, 906, 866, 819, 766, 707, // 0° to 45°
    643,  574, 500, 423, 342, 259, 174, 87,  0         // 50° to 90°
};

// Lookup sin/cos values for an angle (0° to 360°) using the 0°-90° table
static void lookup_sin_cos(int angle, int16_t *sin_val, int16_t *cos_val) {
  // Normalize angle to 0-360
  angle = angle % 360;
  if (angle < 0)
    angle += 360;

  int index = (angle % 90) / 5; // Index into 5° steps (0-18)
  int quadrant = angle / 90;    // Quadrant: 0, 1, 2, 3

  // Base values from table
  int16_t sin_base = sin_table[index];
  int16_t cos_base = cos_table[index];

  // Adjust based on quadrant
  switch (quadrant) {
  case 0: // 0° to 90°
    *sin_val = sin_base;
    *cos_val = cos_base;
    break;
  case 1: // 90° to 180°
    *sin_val = cos_base;
    *cos_val = -sin_base;
    break;
  case 2: // 180° to 270°
    *sin_val = -sin_base;
    *cos_val = -cos_base;
    break;
  case 3: // 270° to 360°
    *sin_val = -cos_base;
    *cos_val = sin_base;
    break;
  }
}

// Rotate XY coordinates using precomputed sin/cos values
static void rotate_xy(int32_t *x, int32_t *y, int16_t sin_val,
                      int16_t cos_val) {
  // Apply rotation: [x'] = [cosθ  -sinθ] [x]
  //                 [y']   [sinθ   cosθ] [y]
  int32_t new_x = (*x * cos_val - *y * sin_val) / 1000;
  int32_t new_y = (*x * sin_val + *y * cos_val) / 1000;

  *x = new_x;
  *y = new_y;
}

// Process input events (implements input_processor.h interface)
static int sensor_rotation_handle_event(
    const struct device *dev, struct input_event *event, uint32_t param1,
    uint32_t param2, struct zmk_input_processor_state *state) {
  struct sensor_rotation_data *data = dev->data;
  const struct sensor_rotation_config *cfg = dev->config;

  switch (event->type) {
  case INPUT_EV_REL:
    if (event->code == INPUT_REL_X) {
      int32_t temp_x = event->value; // Store original input
      int32_t temp_y = data->y;      // Use current y value
      rotate_xy(&temp_x, &temp_y, data->sin_val, data->cos_val);
      data->x = event->value; // Update stored x
      event->value = temp_x;  // Output rotated x
      LOG_DBG("X value: %d, rotate %d : %d, %d : sin %d, cos %d", event->value,
              cfg->rotation_angle, data->x, data->y, data->sin_val,
              data->cos_val);
    } else if (event->code == INPUT_REL_Y) {
      int32_t temp_x = data->x;      // Use current x value
      int32_t temp_y = event->value; // Store original input
      rotate_xy(&temp_x, &temp_y, data->sin_val, data->cos_val);
      data->y = event->value; // Update stored y
      event->value = temp_y;  // Output rotated y
      LOG_DBG("Y value: %d, rotate %d : %d, %d : sin %d, cos %d", event->value,
              cfg->rotation_angle, data->x, data->y, data->sin_val,
              data->cos_val);
    }
    break;
  default:
    return ZMK_INPUT_PROC_CONTINUE; // Unsupported event type
  }
  return 0;
}

// Initialize the processor
static int sensor_rotation_init(const struct device *dev) {
  struct sensor_rotation_data *data = dev->data;
  const struct sensor_rotation_config *cfg = dev->config;

  data->x = 0;
  data->y = 0;

  // Precompute sin and cos values based on the fixed angle
  lookup_sin_cos(cfg->rotation_angle, &data->sin_val, &data->cos_val);

  return 0;
}

// Define the input processor api
static struct zmk_input_processor_driver_api sensor_rotation_driver_api = {
    .handle_event = sensor_rotation_handle_event,
};

#define SENSOR_ROTATION_INST(n)                                                \
  static struct sensor_rotation_data sensor_rotation_data_##n = {};            \
  static const struct sensor_rotation_config sensor_rotation_config_##n = {    \
      .rotation_angle = DT_INST_PROP(n, rotation_angle)};                      \
  DEVICE_DT_INST_DEFINE(                                                       \
      n, sensor_rotation_init, NULL, &sensor_rotation_data_##n,                \
      &sensor_rotation_config_##n, POST_KERNEL,                                \
      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &sensor_rotation_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SENSOR_ROTATION_INST)