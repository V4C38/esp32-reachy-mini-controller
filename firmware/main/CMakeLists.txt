idf_component_register(
    SRCS
        "main.c"
        "imu.c"
        "imu_integrate.c"
        "net_wifi.c"
        "net_discovery.c"
        "net_ws.c"
        "pmu.c"
        "ui.c"
        "ui_face.c"
        "ui_settings.c"
        "assets/reachy_eyes.c"
        "assets/reachy_eyes_closed.c"
    INCLUDE_DIRS "." "assets"
)

target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -ffast-math)
