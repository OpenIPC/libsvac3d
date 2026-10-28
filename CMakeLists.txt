cmake_minimum_required(VERSION 3.5...3.31)

project(uavs3d)

option(COMPILE_10BIT "Enable 10bit streams decoding support." OFF)

set(CMAKE_C_STANDARD 99)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

aux_source_directory(./test DIR_SRC_TEST)

add_subdirectory(./source)

add_executable(uavs3dec ${DIR_SRC_TEST})

if (NOT MSVC)
    target_link_libraries(uavs3dec m)
endif()
target_link_libraries(uavs3dec uavs3d)
#target_link_libraries(uavs3dec dl)

add_executable(svac3dump tools/svac3dump.c)
target_include_directories(svac3dump PRIVATE source/svac3)
target_link_libraries(svac3dump uavs3d)

enable_testing()
file(GLOB SVAC3_TEST_STREAMS ${CMAKE_SOURCE_DIR}/tests/streams/*.svac3)
add_executable(svac3patch tools/svac3patch.c)
target_include_directories(svac3patch PRIVATE source/svac3)
target_link_libraries(svac3patch uavs3d)

add_executable(svac3dec tools/svac3dec.c)
target_include_directories(svac3dec PRIVATE source/svac3)
target_link_libraries(svac3dec uavs3d)

add_executable(test_hls tests/test_hls.c)
target_include_directories(test_hls PRIVATE source/svac3)
target_link_libraries(test_hls uavs3d)
add_test(NAME test_hls
         COMMAND test_hls ${CMAKE_SOURCE_DIR}/tests/streams/cv608_synth_1080p_cbr2m.svac3)
add_executable(test_patch tests/test_patch.c)
target_include_directories(test_patch PRIVATE source/svac3)
target_link_libraries(test_patch uavs3d)
add_test(NAME test_patch
         COMMAND test_patch ${CMAKE_SOURCE_DIR}/tests/streams/cv608_synth_1080p_cbr2m.svac3)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
foreach(stream ${SVAC3_TEST_STREAMS})
    get_filename_component(name ${stream} NAME_WE)
    add_test(NAME patch_parse_${name} COMMAND svac3patch ${stream})
    if(EXISTS ${CMAKE_SOURCE_DIR}/tests/golden/${name}.framemd5)
        add_test(NAME recon_md5_${name}
                 COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/check_md5.py
                         $<TARGET_FILE:svac3dec> ${stream} ${CMAKE_SOURCE_DIR}/tests/golden/${name}.framemd5)
    endif()
endforeach()
add_test(NAME hls_dump
         COMMAND ${CMAKE_SOURCE_DIR}/tests/hls_dump.sh $<TARGET_FILE:svac3dump> ${SVAC3_TEST_STREAMS})
