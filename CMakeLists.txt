cmake_minimum_required(VERSION 3.0)

project(watchdog LANGUAGES C)

add_executable(watchdog main.c)

install(TARGETS watchdog RUNTIME DESTINATION bin)
