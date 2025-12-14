idf_component_register(SRCS "main.c" 
                            "i2c_api.c" 
                            "max30102_api.c" 
                            "algorithm.c" 
                            "spi_dummy.c" 
                            "oled_driver.c" 
                            "mpu6050_api.c"  
                        INCLUDE_DIRS "."
                        REQUIRES )