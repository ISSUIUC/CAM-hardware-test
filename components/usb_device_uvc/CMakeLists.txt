idf_component_register(SRCS usb_device_uvc.c
                    INCLUDE_DIRS "include"
                    REQUIRES usb esp_timer)

# idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)

idf_component_get_property(tinyusb_dir espressif__tinyusb COMPONENT_DIR)

target_include_directories(${COMPONENT_LIB} INTERFACE
    "${tinyusb_dir}/src"
)

target_include_directories(${tinyusb_dir} PUBLIC "${COMPONENT_DIR}/tusb")
# if(NOT EXISTS "${CMAKE_SOURCE_DIR}/components/esp_tinyusb/usb_descriptors.c")
#     target_sources(${tusb_lib} PUBLIC "${COMPONENT_DIR}/tusb/usb_descriptors.c")
# else()
#     message(STATUS "Using existing esp_tinyusb/usb_descriptors.c, skipping... (we already have this in another component library)")
# endif()

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})
