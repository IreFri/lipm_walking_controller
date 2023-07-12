add_executable(test_0 test_0.cpp)
target_link_libraries(test_0 PRIVATE CppLinuxSerial::CppLinuxSerial)

add_executable(test_1 test_1.cpp)
target_link_libraries(test_1 PRIVATE Boost::filesystem)

set(TEST_2_SRC
  test_2.cpp
)
add_executable(test_2 ${TEST_2_SRC})

set(TEST_3_SRC
  test_3.cpp
  serial.cc
  serial.h
  list_ports_linux.cc
  unix.cc
  unix.h
)
add_executable(test_3 ${TEST_3_SRC})