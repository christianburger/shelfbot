# ESP32 Firmware Refactoring Guide for Velocity Control

This guide provides the exact code changes needed to refactor the ESP32 firmware from a position-controlled system to a velocity-controlled system. This is a necessary step to make the robot compatible with standard ROS2 navigation tools like Nav2.

---

### **Step 1: Update the Motor Control Header File**

**Goal:** Add the new primary function for velocity control and mark the old position function as deprecated for the main control loop.

**File:** `motor_control.h`

**Action:** In the section `--- ROS 2 Standard Interface (Primary Control) ---`, replace the existing function declarations with the following block.

**REPLACE THIS:**
```c++
// --- ROS 2 Standard Interface (Primary Control) ---
// All position values are in RADIANS. All velocity values are in RADIANS/SEC.
void motor_control_set_position(uint8_t index, double position_rad);
double motor_control_get_position(uint8_t index);
double motor_control_get_velocity(uint8_t index);
```

**WITH THIS:**
```c++
// --- ROS 2 Standard Interface (Primary Control) ---
// All position values are in RADIANS. All velocity values are in RADIANS/SEC.
void motor_control_set_velocity(uint8_t index, double velocity_rad_s);
double motor_control_get_position(uint8_t index);
double motor_control_get_velocity(uint8_t index);

// --- DEPRECATED: Kept for REST API and position-based testing ---
void motor_control_set_position(uint8_t index, double position_rad);
```

---

### **Step 2: Implement the New Velocity Control Logic**

**Goal:** Add the C++ code that translates a velocity command (rad/s) into continuous motor motion.

**File:** `motor_control.cpp`

**Action:** Add the following new function implementation directly after the `motor_control_get_velocity` function.

```c++
// --- Add this new function ---
void motor_control_set_velocity(uint8_t index, double velocity_rad_s) {
    if (index >= NUM_MOTORS || !steppers[index]) return;

    // If velocity is very close to zero, stop the motor.
    if (fabs(velocity_rad_s) < 1e-4) {
        steppers[index]->stopMove();
        return;
    }

    // Convert velocity in rad/s to speed in Hz (steps/s)
    long speed_hz = (long)fabs(velocity_rad_s * RADS_TO_STEPS);

    // Set the motor speed and acceleration
    steppers[index]->setSpeedInHz(speed_hz);
    steppers[index]->setAcceleration(speed_hz / 2); // Use a reasonable acceleration

    // Set the direction and command the motor to run continuously
    if (velocity_rad_s > 0) {
        steppers[index]->runForward();
    } else {
        steppers[index]->runBackward();
    }
}
```

---

### **Step 3: Connect ROS2 Commands to the New Velocity Logic**

**Goal:** Reroute the incoming velocity commands from the `/shelfbot_firmware/set_speed` topic to our new `motor_control_set_velocity` function.

**File:** `shelfbot.cpp`

**Action:** Find the `set_speed_subscription_callback` function and replace its entire body with the new logic.

**REPLACE THIS:**
```c++
void Shelfbot::set_speed_subscription_callback(const void * msin) {
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msin;
    const long MAX_SPEED_HZ = 2200;

    if (msg->data.size > NUM_MOTORS) {
        ESP_LOGW(TAG, "Received set_speed command with %d values, but only %d motors are supported. Ignoring extra values.", msg->data.size, NUM_MOTORS);
    }

    for (size_t i = 0; i < msg->data.size && i < NUM_MOTORS; i++) {
        float speed_in_rad_s = msg->data.data[i];
        long speed_in_hz = (long)(speed_in_rad_s * RADS_TO_STEPS);
        
        if (speed_in_hz > MAX_SPEED_HZ) {
            ESP_LOGW(TAG, "Motor %d calculated speed (%ld Hz) exceeds limit. Capping at %ld Hz.", i, speed_in_hz, MAX_SPEED_HZ);
            speed_in_hz = MAX_SPEED_HZ;
        }

        ESP_LOGI(TAG, "Motor %d speed command: %.2f rad/s -> Setting speed to %ld Hz", i, speed_in_rad_s, speed_in_hz);
        motor_control_set_speed_hz(i, speed_in_hz);
    }
}
```

**WITH THIS:**
```c++
void Shelfbot::set_speed_subscription_callback(const void * msin) {
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msin;

    if (msg->data.size > NUM_MOTORS) {
        ESP_LOGW(TAG, "Received set_speed command with %d values, but only %d motors are supported. Ignoring extra values.", msg->data.size, NUM_MOTORS);
    }

    // This callback now directly controls the motor's continuous velocity.
    for (size_t i = 0; i < msg->data.size && i < NUM_MOTORS; i++) {
        float velocity_in_rad_s = msg->data.data[i];
        ESP_LOGI(TAG, "Motor %d velocity command: %.2f rad/s", i, velocity_in_rad_s);
        motor_control_set_velocity(i, velocity_in_rad_s);
    }
}
```
