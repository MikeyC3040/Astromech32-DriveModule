set(include_dirs
        ".")

set(srcs
        "main.c"
        "sketch.cpp"
        "joysticks.cpp")

set(requires
        "bluepad32"
        "bluepad32_arduino"
        "arduino"
        "btstack"
        "Reeltwo"
        "HumanCyborgRelationsAPI")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "${include_dirs}"
                    REQUIRES "${requires}")
