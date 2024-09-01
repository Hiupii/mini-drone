#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_system.h"
#include "PID.h"
#include "mpu6050.h"
#include "driver/ledc.h"
#include "math.h"

/*----------mpu6050 var----------*/
#define MPU_ADDR (0x68)

#define CONFIG_SDA_GPIO GPIO_NUM_21
#define CONFIG_SCL_GPIO GPIO_NUM_22

static const char *MPU_TAG = "mpu6050_log";

static mpu6050_acceleration_t accel = { 0 };
static mpu6050_rotation_t rotation = { 0 };

static float input_pitch = 0.0f;
static float input_roll = 0.0f;
static float g = 9.81f;

/*----------pid var----------*/
static const char *PID_TAG = "pid_log";

struct pid_controller pitch_control;
pid_t pitch_pid;

struct pid_controller roll_control;
pid_t roll_pid;

float Kp = 2.75;   // Proportional param
float Ki = 0.005;   // Integral param
float Kd = 1.65;   // Derivative param

static float output_pitch = 0.0f;
static float output_roll = 0.0f;
static float setpoint_pitch = 0.0f;
static float setpoint_roll = 0.0f;


/*----------motor var----------*/
#define MOT_1 GPIO_NUM_32
#define MOT_2 GPIO_NUM_33
#define MOT_3 GPIO_NUM_26
#define MOT_4 GPIO_NUM_27

int base_throttle = 4915;
int motor1_pwm = 0;
int motor2_pwm = 0;
int motor3_pwm = 0;
int motor4_pwm = 0;

int constrain(int value, int min_value, int max_value);

void mpu6050_task(void *pvParameters)
{
    mpu6050_dev_t dev = { 0 };

    ESP_ERROR_CHECK(mpu6050_init_desc(&dev, MPU_ADDR, 0, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO));

    while (1)
    {
        esp_err_t res = i2c_dev_probe(&dev.i2c_dev, I2C_DEV_WRITE);
        if (res == ESP_OK)
        {
            ESP_LOGI(MPU_TAG, "Found MPU60x0 device");
            break;
        }
        ESP_LOGE(MPU_TAG, "MPU60x0 not found");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(mpu6050_init(&dev));

    ESP_LOGI(MPU_TAG, "Accel range: %d", dev.ranges.accel);
    ESP_LOGI(MPU_TAG, "Gyro range:  %d", dev.ranges.gyro);

    while (1)
    {
        ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rotation));
        input_pitch = asinf(accel.x * g / g) * 180 / M_PI;
        input_roll = atan2(accel.y, accel.z) * 180 / M_PI;
        // ESP_LOGI(MPU_TAG, "Pitch: %f", input_pitch);
        // ESP_LOGI(MPU_TAG, "Roll : %f", input_roll);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Thiết lập PWM cho động cơ
void setup_pwm() {
    // Cấu hình PWM cho 4 động cơ
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // Độ phân giải PWM
        .freq_hz = 5000,                      // Tần số PWM
        .speed_mode = LEDC_HIGH_SPEED_MODE,   // Chế độ tốc độ cao
        .timer_num = LEDC_TIMER_0             // Sử dụng bộ timer 0
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel[4] = {
        {.channel    = LEDC_CHANNEL_0,
         .duty       = 0,
         .gpio_num   = MOT_1,          // Chân GPIO nối với động cơ 1
         .speed_mode = LEDC_HIGH_SPEED_MODE,
         .hpoint     = 0,
         .timer_sel  = LEDC_TIMER_0},

        {.channel    = LEDC_CHANNEL_1,
         .duty       = 0,
         .gpio_num   = MOT_2,          // Chân GPIO nối với động cơ 2
         .speed_mode = LEDC_HIGH_SPEED_MODE,
         .hpoint     = 0,
         .timer_sel  = LEDC_TIMER_0},

        {.channel    = LEDC_CHANNEL_2,
         .duty       = 0,
         .gpio_num   = MOT_3,          // Chân GPIO nối với động cơ 3
         .speed_mode = LEDC_HIGH_SPEED_MODE,
         .hpoint     = 0,
         .timer_sel  = LEDC_TIMER_0},

        {.channel    = LEDC_CHANNEL_3,
         .duty       = 0,
         .gpio_num   = MOT_4,          // Chân GPIO nối với động cơ 4
         .speed_mode = LEDC_HIGH_SPEED_MODE,
         .hpoint     = 0,
         .timer_sel  = LEDC_TIMER_0},
    };
    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&ledc_channel[i]);
    }
}

void pid_task(void *pvParameters)
{

    pid_sample(pitch_pid, 100);
    pid_sample(roll_pid, 100);
    while(1)
    {
        pid_compute(pitch_pid);
        pid_compute(roll_pid);

        ESP_LOGI(PID_TAG, "Input Pitch: %f, Output Pitch: %f", input_pitch, output_pitch);
        ESP_LOGI(PID_TAG, "Input Roll : %f, Output Roll : %f", input_roll, output_roll);

        motor1_pwm = base_throttle + output_pitch - output_roll;
        motor2_pwm = base_throttle + output_pitch + output_roll;
        motor3_pwm = base_throttle - output_pitch + output_roll;
        motor4_pwm = base_throttle - output_pitch - output_roll;

        motor1_pwm = constrain(motor1_pwm, 0, 8191);  // 13-bit PWM max value is 8191
        motor2_pwm = constrain(motor2_pwm, 0, 8191);
        motor3_pwm = constrain(motor3_pwm, 0, 8191);
        motor4_pwm = constrain(motor4_pwm, 0, 8191);

        // ESP_LOGI(PID_TAG, "mot1_pwm: %d", motor1_pwm);
        // ESP_LOGI(PID_TAG, "mot2_pwm: %d", motor2_pwm);
        // ESP_LOGI(PID_TAG, "mot3_pwm: %d", motor3_pwm);
        // ESP_LOGI(PID_TAG, "mot4_pwm: %d", motor4_pwm);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main()
{
    // task
    ESP_ERROR_CHECK(i2cdev_init());

    pitch_pid = pid_create(&pitch_control, &input_pitch, &output_pitch, &setpoint_pitch, Kd, Ki, Kp);
    roll_pid = pid_create(&roll_control, &input_roll, &output_roll, &setpoint_roll, Kp, Ki, Kd);

    pid_auto(pitch_pid);
    pid_auto(roll_pid);

    xTaskCreate(mpu6050_task, "mpu6050_test", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
    xTaskCreate(pid_task, "pid_task", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
}

// Hàm constrain: giới hạn giá trị trong phạm vi min và max
int constrain(int value, int min_value, int max_value) {
    if (value < min_value) {
        return min_value;
    } else if (value > max_value) {
        return max_value;
    } else {
        return value;
    }
}


/*

// Cấu hình PID
float Kp = 1.0;   // Hệ số Proportional
float Ki = 0.5;   // Hệ số Integral
float Kd = 0.1;   // Hệ số Derivative

PID pitchPID(&input_pitch, &output_pitch, &setpoint_pitch, Kp, Ki, Kd, DIRECT);
PID rollPID(&input_roll, &output_roll, &setpoint_roll, Kp, Ki, Kd, DIRECT);

// Biến lưu giá trị cảm biến và điều khiển
float input_pitch, input_roll;
float output_pitch, output_roll;
float setpoint_pitch = 0.0;  // Mục tiêu cân bằng (góc 0)
float setpoint_roll = 0.0;   // Mục tiêu cân bằng (góc 0)

// Hàm chính
void app_main() {
    // Khởi tạo PWM
    setup_pwm();

    // Vòng lặp chính
    while (true) {
        // Tính toán giá trị PID
        pitchPID.Compute();
        rollPID.Compute();

        // Điều chỉnh động cơ dựa trên output của PID
        // Giả sử output_pitch và output_roll là giá trị PWM cho động cơ
        // Tăng tốc động cơ phía sau khi pitch nghiêng về phía trước
        int motor1_pwm = base_throttle + output_pitch - output_roll;
        int motor2_pwm = base_throttle + output_pitch + output_roll;
        int motor3_pwm = base_throttle - output_pitch + output_roll;
        int motor4_pwm = base_throttle - output_pitch - output_roll;

        // Đảm bảo giá trị PWM nằm trong giới hạn
        motor1_pwm = constrain(motor1_pwm, 0, 8191);  // 13-bit PWM max value is 8191
        motor2_pwm = constrain(motor2_pwm, 0, 8191);
        motor3_pwm = constrain(motor3_pwm, 0, 8191);
        motor4_pwm = constrain(motor4_pwm, 0, 8191);

        // Cập nhật giá trị PWM cho động cơ
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, motor1_pwm);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, motor2_pwm);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, motor3_pwm);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, motor4_pwm);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3);

        // Delay nhỏ để ổn định vòng lặp
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
*/