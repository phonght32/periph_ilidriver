set(srcs
    "periph_ilidriver.c")

set(includes 
    ".")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS ${includes}
                       REQUIRES periph_esp32 fonts colors)

