# Patch to link arduino to espressif__tinyusb

idf_component_register(
    REQUIRES esp_tinyusb
)

idf_component_get_property(tinyusb_dir espressif__tinyusb COMPONENT_DIR)
idf_component_get_property(esp_tinyusb_dir esp_tinyusb COMPONENT_DIR)

target_include_directories(${COMPONENT_LIB} INTERFACE
    "${tinyusb_dir}/src"
    "${esp_tinyusb_dir}/include"
)
