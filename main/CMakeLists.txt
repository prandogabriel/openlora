set(COMPONENT_SRCS "hello_world_main.c" "../components/lora/lora.c" "u8g2_esp32_hal.c" "u8g2_esp32_hal.h")
set(COMPONENT_ADD_INCLUDEDIRS "../components/lora/include")
set(COMPONENT_ADD_INCLUDEDIRS "../components/backoff/include")

register_component()



