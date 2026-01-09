# idf_component_register(SRCS "usb_device_uvc.c" "tusb/usb_descriptors_uvc.c"
#                     INCLUDE_DIRS "include" "tusb"
#                     REQUIRES usb esp_timer espressif__tinyusb)

idf_component_register(SRCS usb_device_uvc.c
                    INCLUDE_DIRS "include"
                    REQUIRES usb esp_timer espressif__tinyusb)


idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)

target_include_directories(${tusb_lib} PUBLIC "${COMPONENT_DIR}/tusb")
target_sources(${tusb_lib} PUBLIC  "${COMPONENT_DIR}/tusb/usb_descriptors_uvc.c")

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})


