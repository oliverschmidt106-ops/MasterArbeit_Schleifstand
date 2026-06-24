# Top-Level CMake für ESP-IDF-Projekt (Target esp32s3)
cmake_minimum_required(VERSION 3.16)

# Eigene Komponenten zusätzlich zu main/ bekanntmachen
set(EXTRA_COMPONENT_DIRS "components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(glasfaser_schleifstand)
