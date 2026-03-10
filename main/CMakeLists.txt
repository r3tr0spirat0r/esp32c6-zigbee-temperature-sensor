# Pobieramy ścieżkę do folderu z pobranymi bibliotekami
set(managed_comps "${CMAKE_SOURCE_DIR}/managed_components")

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS 
        "."
        "${managed_comps}/espressif__esp-zigbee-lib/include"
        "${managed_comps}/espressif__esp-zigbee-lib/include/zcl"
        "${managed_comps}/espressif__esp-zboss-lib/include"
        "${managed_comps}/espressif__ds18b20/include"
        "${managed_comps}/espressif__onewire_bus/include"
    REQUIRES 
        nvs_flash 
        espressif__esp-zigbee-lib 
        espressif__esp-zboss-lib 
        espressif__ds18b20 
        espressif__onewire_bus
)