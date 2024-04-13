idf_component_register(SRCS "espnow_example_main.c"
                    INCLUDE_DIRS ".")
                    
target_link_libraries(${COMPONENT_LIB} ${CMAKE_CURRENT_LIST_DIR}/../../Загрузки/Martos-main/c-library/xtensa-esp32/target/xtensa-esp32-none-elf/debug/libxtensa_esp32_static_lib.a -Wl,--allow-multiple-definition)
