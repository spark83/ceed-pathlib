
# Define your test executable
add_executable(astar_test astar_test.cpp)

target_include_directories(astar_test PUBLIC ${PATH_INCLUDE_DIR})
target_link_libraries(astar_test ceedpath gtest gtest_main)

# Add test
add_test(NAME AStarTest COMMAND
         astar_test)
