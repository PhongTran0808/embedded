#include "mpu6050_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "driver/i2c.h" // Cần thiết cho các lệnh I2C

static const char *MPU_TAG = "MPU6050_DRV";
static const float ACCEL_SCALE_FACTOR = 16384.0f; // Scale factor cho dải +/- 2g

// Hàm I2C Write cho MPU6050
static esp_err_t mpu_write_register(uint8_t reg_addr, uint8_t data)
{
    uint8_t tx_buf[2] = {reg_addr, data};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, tx_buf, 2, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Hàm I2C Read cho MPU6050
static esp_err_t mpu_read_registers(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    
    // Bước 1: Ghi địa chỉ thanh ghi (Không STOP)
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    
    // Bước 2: RESTART và đọc dữ liệu
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpu6050_init(void)
{
    esp_err_t ret;
    
    // 1. Reset chip (PWR_MGMT_1)
    ret = mpu_write_register(MPU6050_REG_PWR_MGMT_1, PWR_MGMT_1_RESET);
    vTaskDelay(pdMS_TO_TICKS(100));
    if (ret != ESP_OK) return ret;
    
    // 2. Tắt chế độ ngủ và chọn Clock (PWR_MGMT_1)
    ret = mpu_write_register(MPU6050_REG_PWR_MGMT_1, PWR_MGMT_1_CLKSEL_X_AXIS);
    if (ret != ESP_OK) return ret;
    
    // 3. Cấu hình dải đo gia tốc +/- 2g (ACCEL_CONFIG - thanh ghi 0x1C)
    ret = mpu_write_register(0x1C, ACCEL_FS_SEL_2G);
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(MPU_TAG, "MPU6050 initialized.");
    return ESP_OK;
}

esp_err_t mpu6050_read_accel(float *ax, float *ay, float *az)
{
    uint8_t raw_data[6]; // Chỉ cần đọc Accel X, Y, Z (6 bytes)
    // Bắt đầu đọc từ thanh ghi Accel X High (0x3B)
    esp_err_t ret = mpu_read_registers(MPU6050_REG_ACCEL_XOUT_H, raw_data, 6);
    
    if (ret == ESP_OK) {
        // Chuyển đổi Big Endian (High Byte đầu tiên) sang giá trị số nguyên 16-bit
        int16_t accel_raw[3];
        accel_raw[0] = (raw_data[0] << 8) | raw_data[1]; // Ax
        accel_raw[1] = (raw_data[2] << 8) | raw_data[3]; // Ay
        accel_raw[2] = (raw_data[4] << 8) | raw_data[5]; // Az
        
        // Chuyển đổi giá trị thô sang đơn vị g
        *ax = (float)accel_raw[0] / ACCEL_SCALE_FACTOR;
        *ay = (float)accel_raw[1] / ACCEL_SCALE_FACTOR;
        *az = (float)accel_raw[2] / ACCEL_SCALE_FACTOR;
    }
    return ret;
}