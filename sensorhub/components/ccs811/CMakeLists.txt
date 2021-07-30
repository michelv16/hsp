cmake_minimum_required(VERSION 3.5)

set(COMPONENT_REQUIRES)
set(COMPONENT_PRIV_REQUIRES)

set(COMPONENT_SRCS "./ccs811/ccs811.c" "./esp8266_wrapper/esp8266_wrapper.c")
set(COMPONENT_ADD_INCLUDEDIRS "./ccs811/" "./esp8266_wrapper/")

register_component()
