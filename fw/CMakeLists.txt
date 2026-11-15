cmake_minimum_required(VERSION 3.22)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Build only what main/ and components/ actually require.
idf_build_set_property(MINIMAL_BUILD ON)
project(gx5s-base)
