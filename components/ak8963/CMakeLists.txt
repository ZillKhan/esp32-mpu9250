idf_component_register(SRCS "ak8963.c"
                    INCLUDE_DIRS "include"
                    REQUIRES i2c-easy mpu9250
                    )
