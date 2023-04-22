cmake_minimum_required(VERSION 3.0)

project(hmacTest)

find_package(OpenSSL REQUIRED)

add_executable(hmacTest gmac.cpp)
target_link_libraries(hmacTest OpenSSL::SSL)
