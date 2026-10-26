// Brushed micro-quad flight controller (ESP-IDF v5.x)
// ESP32-WROOM-32E | LSM6DSM (SPI) | 4x brushed motors
// Gains/mixer/limits ported from Drone.slx. Bench test with props OFF.

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

static const char *TAG = "fc";

// ---------- PINS ----------
#define PIN_IMU_CS    4
#define PIN_IMU_SCLK  21
#define PIN_IMU_MOSI  19
#define PIN_IMU_MISO  18
#define PIN_M1        26
#define PIN_M2        13
#define PIN_M3        33
#define PIN_M4        25

// ---------- LOOP / PWM ----------
#define LOOP_HZ        1000
#define LOOP_US        (1000000 / LOOP_HZ)
#define PWM_FREQ       20000
#define PWM_RES        LEDC_TIMER_10_BIT
#define PWM_MAX        1023

// ---------- LINK ----------
#define WIFI_SSID       "microquad"
#define WIFI_PASS       "flyquad123"
#define WIFI_CHANNEL    6
#define UDP_PORT        4210
#define PKT_MAGIC       0x51554144u
#define LINK_TIMEOUT_US 500000

// ---------- MODEL ----------
#define SIM_OMEGA_MAX  3000.0f                 // Simulink motor saturation (rad/s)
#define SIM_TO_DUTY    (1.0f / SIM_OMEGA_MAX)
#define HOVER_THROTTLE 0.345f                  // from model; measure on hardware

// ---------- SAFETY ----------
#define MAX_TILT_CMD   0.35f
#define CRASH_TILT     1.05f
#define ARM_THR_MAX    0.05f

// ---------- PID (Simulink parallel form w/ derivative filter N) ----------
typedef struct {
    float kp, ki, kd, n;
    float integ, filt;
    float i_lim, out_lim;
} pidc_t;

// PID Controller4 / 3 / 1 from Drone.slx
static pidc_t rollPID  = { -1.53294785605493f * SIM_TO_DUTY, -0.217782758826352f * SIM_TO_DUTY,
                           -2.68640511728312f * SIM_TO_DUTY, 361.393937805181f, 0, 0, 0.10f, 0.30f };
static pidc_t pitchPID = {  1.53294785605493f * SIM_TO_DUTY,  0.217782758826352f * SIM_TO_DUTY,
                            2.68640511728312f * SIM_TO_DUTY, 361.393937805181f, 0, 0, 0.10f, 0.30f };
static pidc_t yawPID   = {  1421.28268044684f * SIM_TO_DUTY,  470.589569781493f  * SIM_TO_DUTY,
                            956.469408662535f * SIM_TO_DUTY, 11.7175649403718f, 0, 0, 0.10f, 0.20f };
// Altitude (PID Controller2, unused - no alt sensor): P=465.55 I=214.21 D=189.88 N=1142.83
// NOTE: roll/pitch gains are very low once scaled to PWM - retune.

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float wrap_pi(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

static void pid_reset(pidc_t *p, float err) {
    p->integ = 0;
    p->filt  = p->kd * err;
}

static float pid_step(pidc_t *p, float err, float dt, bool integrate) {
    float d_out = p->n * (p->kd * err - p->filt);
    float u = p->kp * err + p->ki * p->integ + d_out;
    p->filt += dt * d_out;
    if (integrate) {
        p->integ += err * dt;
        if (p->ki != 0.0f) {
            float lim = p->i_lim / fabsf(p->ki);
            p->integ = clampf(p->integ, -lim, lim);
        }
    }
    return clampf(u, -p->out_lim, p->out_lim);
}

// ---------- SHARED STATE ----------
typedef struct {
    float thr, roll, pitch, yaw;
    bool  arm;
    int64_t last_us;
} cmd_t;

static cmd_t cmd = {0};
static portMUX_TYPE cmd_mux = portMUX_INITIALIZER_UNLOCKED;

static float gyro[3], accel[3], gyro_bias[3];
static float att[3];                   // roll, pitch, yaw (rad)
static volatile bool armed = false;
static bool imu_ok = false;

static spi_device_handle_t imu;
static TaskHandle_t ctrl_task_handle;

// ---------- IMU (LSM6DSM) ----------
#define LSM_WHO_AM_I   0x0F
#define LSM_CTRL1_XL   0x10
#define LSM_CTRL2_G    0x11
#define LSM_CTRL3_C    0x12
#define LSM_CTRL4_C    0x13
#define LSM_OUTX_L_G   0x22
#define LSM_WHOAMI_VAL 0x6A

#define GYRO_SENS  (0.070f * (float)M_PI / 180.0f)  // ±2000 dps
#define ACC_SENS   (0.244e-3f)                      // ±8 g

static void imu_write(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { reg & 0x7F, val };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    spi_device_polling_transmit(imu, &t);
}

static void imu_read(uint8_t reg, uint8_t *buf, int n) {
    uint8_t tx[1 + 12] = { reg | 0x80 };
    uint8_t rx[1 + 12] = {0};
    spi_transaction_t t = { .length = 8 * (n + 1), .tx_buffer = tx, .rx_buffer = rx };
    spi_device_polling_transmit(imu, &t);
    memcpy(buf, rx + 1, n);
}

static bool imu_init(void) {
    spi_bus_config_t bus = {
        .mosi_io_num = PIN_IMU_MOSI, .miso_io_num = PIN_IMU_MISO,
        .sclk_io_num = PIN_IMU_SCLK, .quadwp_io_num = -1, .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED));
    spi_device_interface_config_t dev = {
        .clock_speed_hz = 8 * 1000 * 1000, .mode = 3,
        .spics_io_num = PIN_IMU_CS, .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &imu));

    imu_write(LSM_CTRL3_C, 0x01);    // reset
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t who = 0;
    imu_read(LSM_WHO_AM_I, &who, 1);
    if (who != LSM_WHOAMI_VAL) {
        ESP_LOGE(TAG, "LSM6DSM WHO_AM_I = 0x%02X (expected 0x6A)", who);
        return false;
    }
    imu_write(LSM_CTRL3_C,  0x44);   // BDU, IF_INC
    imu_write(LSM_CTRL4_C,  0x04);   // I2C off
    imu_write(LSM_CTRL1_XL, 0x8C);   // 1.66 kHz, ±8 g
    imu_write(LSM_CTRL2_G,  0x8C);   // 1.66 kHz, ±2000 dps
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "LSM6DSM OK");
    return true;
}

// Chip -> body (NED). Assumes chip face up, X toward nose.
static void chip_to_body(const float c[3], float b[3]) {
    b[0] =  c[0];
    b[1] = -c[1];
    b[2] = -c[2];
}

static void imu_sample(void) {
    uint8_t b[12];
    imu_read(LSM_OUTX_L_G, b, 12);
    int16_t raw[6];
    for (int i = 0; i < 6; i++) raw[i] = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8));
    float g_chip[3] = { raw[0] * GYRO_SENS, raw[1] * GYRO_SENS, raw[2] * GYRO_SENS };
    float a_chip[3] = { raw[3] * ACC_SENS,  raw[4] * ACC_SENS,  raw[5] * ACC_SENS  };
    chip_to_body(g_chip, gyro);
    chip_to_body(a_chip, accel);
    for (int i = 0; i < 3; i++) gyro[i] -= gyro_bias[i];
}

// ---------- ATTITUDE (complementary filter) ----------
#define CF_TAU 0.5f

static void accel_angles(float *roll, float *pitch) {
    *roll  = atan2f(-accel[1], -accel[2]);
    *pitch = atan2f(accel[0], sqrtf(accel[1] * accel[1] + accel[2] * accel[2]));
}

static void attitude_update(float dt) {
    float p = gyro[0], q = gyro[1], r = gyro[2];
    float sphi = sinf(att[0]), cphi = cosf(att[0]);
    float cth  = cosf(att[1]);
    if (fabsf(cth) < 0.05f) cth = copysignf(0.05f, cth);
    float tth  = sinf(att[1]) / cth;

    att[0] += (p + (q * sphi + r * cphi) * tth) * dt;
    att[1] += (q * cphi - r * sphi) * dt;
    att[2]  = wrap_pi(att[2] + ((q * sphi + r * cphi) / cth) * dt);

    float an = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    if (an > 0.85f && an < 1.15f) {
        float ar, ap;
        accel_angles(&ar, &ap);
        const float A = CF_TAU / (CF_TAU + dt);
        att[0] = A * att[0] + (1.0f - A) * ar;
        att[1] = A * att[1] + (1.0f - A) * ap;
    }
}

// ---------- MIXER (Simulink Sum3..Sum6) ----------
// M1 front-left CW, M2 front-right CCW, M3 rear-right CW, M4 rear-left CCW
static void motors_write(float thr, float r, float pch, float yw) {
    float m[4] = {
        thr - r + pch - yw,
        thr + r + pch + yw,
        thr + r - pch - yw,
        thr - r - pch + yw
    };
    for (int i = 0; i < 4; i++) {
        int duty = (int)(clampf(m[i], 0.0f, 1.0f) * PWM_MAX);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
    }
}

static void motors_off(void) { motors_write(0, 0, 0, 0); }

// ---------- CONTROL LOOP (core 1) ----------
static void loop_timer_cb(void *arg) {
    xTaskNotifyGive(ctrl_task_handle);
}

static void calibrate_gyro(void) {
    const int N = 2000;
    float sum[3] = {0};
    memset(gyro_bias, 0, sizeof(gyro_bias));
    for (int i = 0; i < N; i++) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        imu_sample();
        for (int k = 0; k < 3; k++) sum[k] += gyro[k];
    }
    for (int k = 0; k < 3; k++) gyro_bias[k] = sum[k] / N;
    accel_angles(&att[0], &att[1]);
    att[2] = 0;
    ESP_LOGI(TAG, "gyro bias %.4f %.4f %.4f", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}

static void control_task(void *arg) {
    motors_off();
    if (!imu_ok) {
        ESP_LOGE(TAG, "IMU init failed, motors off");
        vTaskDelete(NULL);
    }
    calibrate_gyro();

    int64_t prev = esp_timer_get_time();
    float yaw_ref = 0;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int64_t now = esp_timer_get_time();
        float dt = clampf((now - prev) * 1e-6f, 0.5f / LOOP_HZ, 3.0f / LOOP_HZ);
        prev = now;

        imu_sample();
        attitude_update(dt);

        cmd_t c;
        portENTER_CRITICAL(&cmd_mux);
        c = cmd;
        portEXIT_CRITICAL(&cmd_mux);
        bool link_ok = (now - c.last_us) < LINK_TIMEOUT_US;

        if (!armed) {
            if (c.arm && link_ok && c.thr < ARM_THR_MAX &&
                fabsf(att[0]) < 0.3f && fabsf(att[1]) < 0.3f) {
                yaw_ref = att[2];
                pid_reset(&rollPID,  -att[0]);
                pid_reset(&pitchPID, -att[1]);
                pid_reset(&yawPID,   0);
                armed = true;
            }
        } else if (!c.arm || !link_ok || fabsf(att[0]) > CRASH_TILT || fabsf(att[1]) > CRASH_TILT) {
            armed = false;
        }

        if (armed) {
            float sp_roll  = clampf(c.roll,  -MAX_TILT_CMD, MAX_TILT_CMD);
            float sp_pitch = clampf(c.pitch, -MAX_TILT_CMD, MAX_TILT_CMD);
            float sp_yaw   = wrap_pi(yaw_ref + c.yaw);
            bool airborne  = c.thr > 0.5f * HOVER_THROTTLE;

            float r = pid_step(&rollPID,  sp_roll  - att[0],        dt, airborne);
            float p = pid_step(&pitchPID, sp_pitch - att[1],        dt, airborne);
            float y = pid_step(&yawPID,   wrap_pi(sp_yaw - att[2]), dt, airborne);
            motors_write(clampf(c.thr, 0.0f, 1.0f), r, p, y);
        } else {
            motors_off();
        }
    }
}

// ---------- WIFI (core 0) ----------
// Packet: struct.pack('<IIBffff', 0x51554144, seq, arm, thr, roll, pitch, yaw) -> 192.168.4.1:4210
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t seq;
    uint8_t  arm;
    float    throttle, roll, pitch, yaw;
} ctrl_pkt_t;

static void wifi_init_softap(void) {
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wc = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASS,
            .max_connection = 1,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "AP '%s' up, UDP %d", WIFI_SSID, UDP_PORT);
}

static void wifi_task(void *arg) {
    wifi_init_softap();

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    bind(sock, (struct sockaddr *)&addr, sizeof(addr));
    struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint32_t last_seq = 0;
    for (;;) {
        ctrl_pkt_t pkt;
        int n = recvfrom(sock, &pkt, sizeof(pkt), 0, NULL, NULL);
        if (n != sizeof(pkt) || pkt.magic != PKT_MAGIC) continue;
        if (!isfinite(pkt.throttle) || !isfinite(pkt.roll) ||
            !isfinite(pkt.pitch) || !isfinite(pkt.yaw)) continue;

        int64_t now = esp_timer_get_time();
        portENTER_CRITICAL(&cmd_mux);
        bool stale = (now - cmd.last_us) > LINK_TIMEOUT_US;
        if (stale || (int32_t)(pkt.seq - last_seq) > 0) {
            last_seq  = pkt.seq;
            cmd.arm   = pkt.arm != 0;
            cmd.thr   = clampf(pkt.throttle, 0.0f, 1.0f);
            cmd.roll  = pkt.roll;
            cmd.pitch = pkt.pitch;
            cmd.yaw   = clampf(pkt.yaw, -(float)M_PI, (float)M_PI);
            cmd.last_us = now;
        }
        portEXIT_CRITICAL(&cmd_mux);
    }
}

static void telemetry_task(void *arg) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));
        printf("armed=%d  roll=%6.1f  pitch=%6.1f  yaw=%6.1f deg\n", armed,
               att[0] * 57.2958f, att[1] * 57.2958f, att[2] * 57.2958f);
    }
}

// ---------- INIT ----------
static void pwm_init(void) {
    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = PWM_RES,
        .timer_num = LEDC_TIMER_0, .freq_hz = PWM_FREQ, .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));
    int pins[4] = { PIN_M1, PIN_M2, PIN_M3, PIN_M4 };
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t c = {
            .speed_mode = LEDC_LOW_SPEED_MODE, .channel = (ledc_channel_t)i,
            .timer_sel = LEDC_TIMER_0, .gpio_num = pins[i], .duty = 0, .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&c));
    }
}

void app_main(void) {
    pwm_init();
    imu_ok = imu_init();

    xTaskCreatePinnedToCore(control_task,   "ctrl", 8192, NULL, configMAX_PRIORITIES - 2,
                            &ctrl_task_handle, 1);
    xTaskCreatePinnedToCore(wifi_task,      "wifi", 8192, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(telemetry_task, "tlm",  4096, NULL, 1, NULL, 0);

    const esp_timer_create_args_t targs = { .callback = loop_timer_cb, .name = "loop" };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&targs, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, LOOP_US));
}
