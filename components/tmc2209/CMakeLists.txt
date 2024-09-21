# Set the component name
set(COMPONENT_NAME tmc2209)

# Add source files
set(COMPONENT_SRCS 
    tmc2209.c
)

message("Component registered. Target name: ${COMPONENT_TARGET}")
# Add include directories (if any)
set(COMPONENT_ADD_INCLUDEDIRS 
    include         # If you have header files in an 'include' directory
)

# Register the component
# idf_component_register(SRCS ${COMPONENT_SRCS} 
#                        INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS})


target_link_libraries(tmc2209 PRIVATE driver/gpio)