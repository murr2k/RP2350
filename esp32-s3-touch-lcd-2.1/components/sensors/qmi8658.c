#include "qmi8658.h"

#include "board_config.h"
#include "dev_config.h"
#include "esp_log.h"

static const char *TAG = "qmi8658";

/* CTRL1: address auto-increment on, little endian, sensors powered.
 * Auto-increment is what lets the six sample registers come back in one burst
 * instead of twelve single-register reads like the RP2350 demos did. */
#define CTRL1_ADDR_AUTO_INC 0x40

/* CTRL2: accelerometer full scale +/-2 g (bits 6:4 = 000), ODR 250 Hz (0x5).
 * CTRL3: gyroscope full scale +/-256 dps (bits 6:4 = 100), ODR 250 Hz (0x5).
 *
 * The RP2350 sources wrote 0x04 / 0x54 here while dividing by 16384 and 128.
 * 0x54 actually selects +/-512 dps, so the gyro readings there came out at half
 * the true rate. The values below make the hardware match the scale factors
 * that the demos, the README and the host test scripts all assume. */
#define CTRL2_ACC_2G_250HZ    0x05
#define CTRL3_GYRO_256DPS_250HZ 0x45

/* CTRL5: low pass filters enabled for both sensors, mode 0 (2.66% of ODR). */
#define CTRL5_LPF_BOTH 0x11

/* CTRL7: enable accelerometer and gyroscope. */
#define CTRL7_ENABLE_ACC_GYRO 0x03

#define QMI8658_SOFT_RESET 0xB0

static uint8_t s_addr;
static bool s_present;
static vector3f_t s_gyro_offset;

static bool detect(void)
{
    const uint8_t candidates[] = {BOARD_IMU_ADDR, BOARD_IMU_ADDR_ALT};

    for (size_t i = 0; i < sizeof(candidates); i++) {
        const uint8_t addr = candidates[i];
        if (!DEV_I2C_Probe(addr)) {
            continue;
        }
        const uint8_t who = DEV_I2C_Read_Byte(addr, QMI8658_WHO_AM_I);
        if (who == QMI8658_DEVICE_ID) {
            s_addr = addr;
            return true;
        }
        ESP_LOGW(TAG, "device at 0x%02x answered WHO_AM_I 0x%02x, expected 0x%02x",
                 addr, who, QMI8658_DEVICE_ID);
    }
    return false;
}

bool qmi8658_init(void)
{
    s_present = false;
    s_gyro_offset = (vector3f_t){0.0f, 0.0f, 0.0f};

    if (!detect()) {
        ESP_LOGE(TAG, "no QMI8658 found on the I2C bus");
        return false;
    }

    DEV_I2C_Write_Byte(s_addr, QMI8658_RESET, QMI8658_SOFT_RESET);
    DEV_Delay_ms(20);

    DEV_I2C_Write_Byte(s_addr, QMI8658_CTRL1, CTRL1_ADDR_AUTO_INC);
    DEV_Delay_ms(10);
    DEV_I2C_Write_Byte(s_addr, QMI8658_CTRL2, CTRL2_ACC_2G_250HZ);
    DEV_I2C_Write_Byte(s_addr, QMI8658_CTRL3, CTRL3_GYRO_256DPS_250HZ);
    DEV_I2C_Write_Byte(s_addr, QMI8658_CTRL5, CTRL5_LPF_BOTH);
    DEV_I2C_Write_Byte(s_addr, QMI8658_CTRL7, CTRL7_ENABLE_ACC_GYRO);
    DEV_Delay_ms(10);

    const uint8_t revision = DEV_I2C_Read_Byte(s_addr, QMI8658_REVISION_ID);
    ESP_LOGI(TAG, "ready at 0x%02x, revision 0x%02x, +/-2 g and +/-256 dps at 250 Hz",
             s_addr, revision);

    s_present = true;
    return true;
}

bool qmi8658_present(void)
{
    return s_present;
}

uint8_t qmi8658_address(void)
{
    return s_present ? s_addr : 0;
}

bool qmi8658_read_raw(int16_t *acc_raw, int16_t *gyro_raw)
{
    if (!s_present) {
        return false;
    }

    uint8_t buf[12];
    if (DEV_I2C_Read_nByte(s_addr, QMI8658_AX_L, buf, sizeof(buf)) != ESP_OK) {
        return false;
    }

    for (int i = 0; i < 3; i++) {
        acc_raw[i] = (int16_t)((uint16_t)buf[i * 2 + 1] << 8 | buf[i * 2]);
        gyro_raw[i] = (int16_t)((uint16_t)buf[6 + i * 2 + 1] << 8 | buf[6 + i * 2]);
    }
    return true;
}

bool qmi8658_read(vector3f_t *acc, vector3f_t *gyro)
{
    int16_t acc_raw[3];
    int16_t gyro_raw[3];

    if (!qmi8658_read_raw(acc_raw, gyro_raw)) {
        return false;
    }

    if (acc != NULL) {
        acc->x = acc_raw[0] / QMI8658_ACC_LSB_PER_G;
        acc->y = acc_raw[1] / QMI8658_ACC_LSB_PER_G;
        acc->z = acc_raw[2] / QMI8658_ACC_LSB_PER_G;
    }
    if (gyro != NULL) {
        gyro->x = gyro_raw[0] / QMI8658_GYRO_LSB_PER_DPS - s_gyro_offset.x;
        gyro->y = gyro_raw[1] / QMI8658_GYRO_LSB_PER_DPS - s_gyro_offset.y;
        gyro->z = gyro_raw[2] / QMI8658_GYRO_LSB_PER_DPS - s_gyro_offset.z;
    }
    return true;
}

float qmi8658_read_temperature(void)
{
    if (!s_present) {
        return 0.0f;
    }
    uint8_t buf[2];
    if (DEV_I2C_Read_nByte(s_addr, QMI8658_TEMP_L, buf, sizeof(buf)) != ESP_OK) {
        return 0.0f;
    }
    return (float)buf[1] + (float)buf[0] / 256.0f;
}

void qmi8658_calibrate(uint16_t samples)
{
    if (!s_present || samples == 0) {
        return;
    }

    s_gyro_offset = (vector3f_t){0.0f, 0.0f, 0.0f};

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    uint16_t taken = 0;

    for (uint16_t i = 0; i < samples; i++) {
        vector3f_t gyro;
        if (qmi8658_read(NULL, &gyro)) {
            sum_x += gyro.x;
            sum_y += gyro.y;
            sum_z += gyro.z;
            taken++;
        }
        DEV_Delay_ms(5);
    }

    if (taken == 0) {
        return;
    }
    s_gyro_offset.x = sum_x / taken;
    s_gyro_offset.y = sum_y / taken;
    s_gyro_offset.z = sum_z / taken;
}

vector3f_t qmi8658_gyro_offset(void)
{
    return s_gyro_offset;
}

void qmi8658_set_gyro_offset(vector3f_t offset)
{
    s_gyro_offset = offset;
}

bool qmi8658_data_ready(void)
{
    if (!s_present) {
        return false;
    }
    return (DEV_I2C_Read_Byte(s_addr, QMI8658_STATUS0) & 0x03) != 0;
}
