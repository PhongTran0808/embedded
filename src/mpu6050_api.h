#ifndef MPU6050_API_H
#define MPU6050_API_H

#include "esp_err.h"
#include <stdint.h>
#include "i2c_api.h" // Sử dụng I2C Bus Driver đã có

// Địa chỉ MPU6050
#define MPU6050_ADDR 0x68 

// Các thanh ghi cần thiết
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

// Cấu hình
#define PWR_MGMT_1_RESET 0x80
#define PWR_MGMT_1_CLKSEL_X_AXIS 0x01
#define ACCEL_FS_SEL_2G 0x00 // Chọn dải đo +/- 2g

// Khai báo hàm
esp_err_t mpu6050_init(void);
esp_err_t mpu6050_read_accel(float *ax, float *ay, float *az);
// Có thể thêm hàm đọc gyro nếu cần: esp_err_t mpu6050_read_gyro(float *gx, float *gy, float *gz);

#endif