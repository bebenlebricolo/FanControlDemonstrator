
######################################################################
############################ Bridge tests ############################
######################################################################
macro( declare_core_test TEST_NAME SOURCES )

    add_executable(${TEST_NAME}
        ${SOURCES}
    )

    gtest_discover_tests(${TEST_NAME} )

    target_include_directories(${TEST_NAME}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/..
    )

    target_link_libraries(${TEST_NAME}
        core
        GTest::gtest
    )

    set_target_properties(${TEST_NAME}
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/tests"
    )

endmacro()

declare_core_test(bridge_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/bridge_tests.cpp
)

declare_core_test(interpolation_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/interpolation_tests.cpp
)

declare_core_test(thermistor_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/thermistor_tests.cpp
)

declare_core_test(current_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/current_tests.cpp
)

declare_core_test(led_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/led_tests.cpp
)

declare_core_test(spanner_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/spanner_tests.cpp
)

declare_core_test(sliding_average_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/sliding_average_tests.cpp
)