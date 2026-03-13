idf_component_register(
    SRCS 
        "main.c"
        "quaternion.c"
        "imu/imu.c"
        "imu/bmi088/bmi088_esp32_interface.c"
        "imu/bmi088/bmi088_utils.c"
        "imu/bmi088/bosch/bmi08a.c"
        "imu/bmi088/bosch/bmi08g.c"
        "imu/bmi088/bosch/bmi08xa.c"

    INCLUDE_DIRS 
        "."
        "imu"
        "imu/bmi088"
        "imu/bmi088/bosch"
)
