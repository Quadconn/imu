idf_component_register(
    SRCS 
        "main.c"
        "bmi088_esp32_interface.c"
        "bmi088/bmi08a.c"
        "bmi088/bmi08g.c"
        "bmi088/bmi08xa.c"

    INCLUDE_DIRS 
        "."
        "bmi088"
)
