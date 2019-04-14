set(EXT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/ext)
set (TEST_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/tests)
include_directories(${EXT_FOLDER} ${CMAKE_CURRENT_SOURCE_DIR})

set(HEADERS
    accel.h
    closestpointquery.h
    common.h
    geometry.h
    matrix.h
    mesh.h
    queryfunctions.h
    sampling.h
    util.h
    vector3.h
    )

set (SRCS
    accel.cpp
    closestpointquery.cpp
    main.cpp
    mesh.cpp
    util.cpp)

set (TEST_SRCS
     ${TEST_FOLDER}/test_geometry.cpp
     ${TEST_FOLDER}/test_matrix.cpp
     ${TEST_FOLDER}/test_queryfunctions.cpp
     ${TEST_FOLDER}/test_sampling.cpp
     ${TEST_FOLDER}/test_util.cpp
     ${TEST_FOLDER}/test_vector.cpp)

set(EXT_SRCS
    ${EXT_FOLDER}/catch.hpp
    ${EXT_FOLDER}/ProgramOptions.hxx
    ${EXT_FOLDER}/tiny_obj_loader.h)

source_group(ext FILES ${EXT_SRCS})
add_executable(closestpoint ${HEADERS} ${SRCS} ${EXT_SRCS})
add_executable(testsuite ${TEST_SRCS})

if(MSVC)
  target_compile_options(closestpoint PRIVATE /W4 /WX)
  target_compile_options(testsuite PRIVATE /W4 /WX)
else(MSVC)
  target_link_libraries(closestpoint "-pthread")
  target_compile_options(closestpoint PRIVATE -Wall -Wextra -pedantic -Werror)
  target_compile_options(testsuite PRIVATE -Wall -Wextra -pedantic -Werror)
endif(MSVC)


INSTALL(TARGETS closestpoint testsuite DESTINATION ${INSTALL_DIR})
