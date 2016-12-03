cmake_minimum_required(VERSION 2.8.7)
project(nrf24xx_core)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror")

set(LIB_SRCS src/nrf24xx.c)

add_library(nrf24xx_core SHARED ${LIB_SRCS})
include_directories(inc)
set_target_properties(nrf24xx_core PROPERTIES PUBLIC_HEADER "inc/nrf24l01_defines.h;inc/nrf24xx.h")
set_target_properties(nrf24xx_core PROPERTIES SOVERSION "1")

install(TARGETS nrf24xx_core DESTINATION lib PUBLIC_HEADER DESTINATION inc)

### Debian Package generation
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_VERSION "0.1.0")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Franz Flasch") #required
include(CPack)
