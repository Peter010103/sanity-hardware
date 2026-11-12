cmake_minimum_required(VERSION 3.16)
set(IDF_TARGET esp32s3 CACHE STRING "Sanity receiver target")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sanity_zenoh_receiver)

# ESP-Hosted 2.12 initializes from a constructor by default. The receiver owns
# its lifecycle so the USB passthrough boot path does not start the radio.
if(IDF_TARGET STREQUAL "esp32p4")
    idf_component_get_property(hosted_lib espressif__esp_hosted COMPONENT_LIB)
    get_target_property(hosted_sources ${hosted_lib} SOURCES)
    list(FILTER hosted_sources EXCLUDE REGEX "port_esp_hosted_host_init[.]c$")
    set_property(TARGET ${hosted_lib} PROPERTY SOURCES "${hosted_sources}")
endif()
