#include "freertos/FreeRTOS.h"
#include "freertos/task.h" 
#include "esp_system.h"
#include "nvs_flash.h"
#include "main.h" 
#include "max30102_api.h"
#include "algorithm.h" 
#include "i2c_api.h" 
#include "oled_driver.h" 
#include "mpu6050_api.h" 
#include "driver/gpio.h" 
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h> 
#include "esp_log.h" 
#include <math.h> 
#include <string.h> 

static const char *TAG = "MAX30102_APP";

// =========================================================
// CẤU HÌNH VÀ BIẾN GLOBAL
// =========================================================

#define BUZZER_GPIO 5 // GPIO 5 dùng để cấp VCC cho Buzzer
// >>> ĐÃ GIẢM NGƯỠNG ĐỂ DỄ DÀNG LÀM DEMO HƠN <<<
#define ACCEL_THRESHOLD 1.1f // Ngưỡng gia tốc để phát hiện chuyển động (1.1 lần lực hấp dẫn)

int32_t red_data = 0;
int32_t ir_data = 0;
int32_t red_data_buffer[BUFFER_SIZE]; 
int32_t ir_data_buffer[BUFFER_SIZE];
double auto_correlationated_data[BUFFER_SIZE];

#define DELAY_AMOSTRAGEM 40
#define CYCLE_DELAY_MS 500 

Oled_t oled_dev; 

static int heart_frame_counter = 0; 
static float g_hrv_rmssd = 0.0f;
static char g_stress_status[16] = "N/A"; 

// Biến MPU6050
static float g_accel_x = 0.0f;
static float g_accel_y = 0.0f;
static float g_accel_z = 0.0f;
static float g_total_accel = 0.0f; // Gia tốc tổng

// =========================================================
// KHAI BÁO HÀM CỤC BỘ
// =========================================================
void fill_buffers_data();
void sensor_data_reader(void *pvParameters);

/**
 * @brief Kích hoạt Buzzer cho một lần bíp ngắn
 */
void trigger_buzzer(int duration_ms) {
    gpio_set_level(BUZZER_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    gpio_set_level(BUZZER_GPIO, 0);
}


void display_task_values(int heart_rate, double spo2, double correlation) {
    char buffer[32];
    
    oled_clear_screen(&oled_dev); 
    
    int current_frame = heart_frame_counter % 2; 

    if (correlation >= 0.7 && heart_rate >= 40 && spo2 > 80.0) {
        
        // Dòng 1: HR & Tim đập
        oled_draw_heart_animation(&oled_dev, 0, 13, current_frame); 
        sprintf(buffer, "HR: %d bpm", heart_rate);
        oled_draw_text(&oled_dev, 0, 0, buffer); 

        // Dòng 2: SpO2
        sprintf(buffer, "SpO2: %.1f %%", spo2);
        oled_draw_text(&oled_dev, 1, 0, buffer); 
        
        // Dòng 3: HRV
        if (g_hrv_rmssd > 0.0f) {
            sprintf(buffer, "HRV: %.1f ms", g_hrv_rmssd);
        } else {
            sprintf(buffer, "HRV: N/A");
        }
        oled_draw_text(&oled_dev, 2, 0, buffer);
        
        // Dòng 4: Trạng thái cơ thể
        sprintf(buffer, "Status: %s", g_stress_status);
        oled_draw_text(&oled_dev, 3, 0, buffer); 
        
        // Dòng 5: Gia tốc (Accel Z)
        sprintf(buffer, "AccZ: %.1f g (T:%.1f)", g_accel_z, g_total_accel);
        oled_draw_text(&oled_dev, 4, 0, buffer); 

    } else {
        // Cảnh báo
        oled_draw_text(&oled_dev, 0, 0, "Input Your Finger"); 
        
        if (correlation > 0.0) {
            sprintf(buffer, "Low Qual: %.1f", correlation);
            oled_draw_text(&oled_dev, 1, 0, buffer); 
        } else {
            oled_draw_text(&oled_dev, 1, 0, "Waiting for signal");
        }
    }
    
    oled_update_display(&oled_dev); 
}

void app_main(void)
{
    // 1. Khởi tạo NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 2. Khởi tạo Buzzer GPIO (GPIO 5)
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_GPIO, 0); 
    
    // 3. Khởi tạo I2C Bus Driver
    i2c_bus_driver_install(); 

    // 4. Khởi tạo Sensors
    ESP_LOGI(TAG, "Initializing MAX30102...");
    max30102_init(&max30102_configuration); 
    
    ESP_LOGI(TAG, "Initializing MPU6050...");
    mpu6050_init(); 

    ESP_LOGI(TAG, "Initializing OLED...");
    if(oled_init(&oled_dev) != ESP_OK) {
        ESP_LOGE(TAG, "OLED initialization FAILED! Check address/Bus integrity.");
    }
    
    oled_draw_text(&oled_dev, 0, 0, "System Ready"); 
    oled_update_display(&oled_dev); 
    
    vTaskDelay(pdMS_TO_TICKS(500)); 

    ESP_LOGI(TAG, "Starting sensor reader task on Core 1");
    xTaskCreatePinnedToCore(sensor_data_reader, "Data", 10240, NULL, 2, NULL, 1);
}


void sensor_data_reader(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(100)); 
    
    init_time_array();
    
    uint64_t ir_mean;
    uint64_t red_mean;
    float temperature;
    double r0_autocorrelation;

    for(;;){
        // A. Thu thập dữ liệu MAX30102
        fill_buffers_data();

        // B. Thu thập dữ liệu MPU-6050
        mpu6050_read_accel(&g_accel_x, &g_accel_y, &g_accel_z); 
        
        // Tính gia tốc tổng (Vector Magnitude)
        g_total_accel = sqrtf(g_accel_x*g_accel_x + g_accel_y*g_accel_y + g_accel_z*g_accel_z);


        // C. PHÁT HIỆN GIA TỐC VÀ KÍCH HOẠT ALARM (5 giây)
        if (g_total_accel > ACCEL_THRESHOLD) {
            ESP_LOGW(TAG, "!!!!! MOTION DETECTED !!!!! (Total Accel: %.2f g)", g_total_accel);
            
            // Tạo 5 giây bíp bíp liên tục (5000 ms)
            for (int i = 0; i < 25; i++) { // 25 cycles * 200ms/cycle = 5000ms
                gpio_set_level(BUZZER_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(100)); // Beep 100ms
                gpio_set_level(BUZZER_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(100)); // Silence 100ms
            }
            ESP_LOGW(TAG, "Alarm sequence completed.");

        } else {
             // Log gia tốc để dễ quan sát
            ESP_LOGI(TAG, "MPU: Accel (X:%.2f, Y:%.2f, Z:%.2f, Total:%.2f g)", g_accel_x, g_accel_y, g_accel_z, g_total_accel);
        }
        
        // D. Xử lý dữ liệu Sinh lý (HR/SpO2/HRV)
        temperature = get_max30102_temp();
        remove_dc_part(ir_data_buffer, red_data_buffer, &ir_mean, &red_mean);
        remove_trend_line(ir_data_buffer);
        remove_trend_line(red_data_buffer);
        
        double pearson_correlation = correlation_datay_datax(red_data_buffer, ir_data_buffer);
        int heart_rate = calculate_heart_rate(ir_data_buffer, &r0_autocorrelation, auto_correlationated_data);
        
        bool is_hr_valid = (heart_rate >= 40 && heart_rate <= 200);

        if(pearson_correlation >= 0.7 && is_hr_valid){ 
            double spo2 = spo2_measurement(ir_data_buffer, red_data_buffer, ir_mean, red_mean);
            
            // TÍNH HRV VÀ DỰ ĐOÁN STRESS
            g_hrv_rmssd = calculate_hrv_rmssd(ir_data_buffer, BUFFER_SIZE);
            predict_stress(heart_rate, spo2, g_hrv_rmssd, g_stress_status);

            // Cảnh báo Stress (Bíp ngắn)
            if (strcmp(g_stress_status, "Stress") == 0) {
                 trigger_buzzer(50); 
            }

            printf("\n| MEASUREMENT (HR:%d, Status:%s, Accel Total:%.1f g)\n", heart_rate, g_stress_status, g_total_accel);
            
            heart_frame_counter++; 
            display_task_values(heart_rate, spo2, pearson_correlation); 

        } else {
             printf("\n| WARNING: Low signal quality (Corr: %.2f). Temp:%.2f.\n", pearson_correlation, temperature);
             
             // Reset trạng thái nếu tín hiệu kém
             strcpy(g_stress_status, "N/A");
             g_hrv_rmssd = 0.0f;
             
             display_task_values(0, 0.0, pearson_correlation); 
        }
        
        vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_MS));
    }
}


void fill_buffers_data()
{
    for(int i = 0; i < BUFFER_SIZE; i++){
        read_max30102_fifo(&red_data, &ir_data);
        ir_data_buffer[i] = ir_data;
        red_data_buffer[i] = red_data;
        
        ir_data = 0;
        red_data = 0;
        vTaskDelay(pdMS_TO_TICKS(DELAY_AMOSTRAGEM));
    }
}