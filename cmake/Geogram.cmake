find_package(Geogram REQUIRED)

if(WIN32) # this macro should normally be defined when configuring the geogram, but it appears not in some cases...
    add_compile_definitions(GEOGRAM_USE_BUILTIN_DEPS)
endif()