file(READ version.txt IO_EXPANDER_VERSION)
message("IO_EXPANDER_VERSION : ${IO_EXPANDER_VERSION}")

# sources
add_library(io_expander INTERFACE)

# include dirs
target_include_directories(io_expander INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

# defines
target_compile_definitions(io_expander INTERFACE -DIO_EXPANDER_VERSION="${IO_EXPANDER_VERSION}")

# depends
target_link_libraries(io_expander INTERFACE etl)
target_link_libraries(io_expander INTERFACE periph)