# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})
link_directories(
	${CMAKE_BINARY_DIR}/src
    )
    include_directories(../loop-functions/complexity)
add_library(example_aggregation_loopfunc SHARED ExampleAggregationLoopFunc.h ExampleAggregationLoopFunc.cpp)

target_link_libraries(example_aggregation_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)



# Single Spot Aggregation

add_library(example_aggregation_single_loopfunc SHARED AggregationSingleSpot.h AggregationSingleSpot.cpp)

target_link_libraries(example_aggregation_single_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)






