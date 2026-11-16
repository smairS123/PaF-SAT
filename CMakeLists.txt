cmake_minimum_required(VERSION 3.21)
project(paf_sat LANGUAGES CXX)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "Use an out-of-source build: cmake -S . -B build")
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# boOX (the NRF-SAT encoder and the Glucose SAT solver) is not part of this repository. Point BOOX_DIR to a boOX
# checkout (the folder that contains boOX-y/) with patches/boox-makespan.patch applied; see the README.
set(BOOX_DIR "" CACHE PATH "boOX checkout with patches/boox-makespan.patch applied")
set(BOOX_SOURCE "${BOOX_DIR}/boOX-y/src")
if(NOT EXISTS "${BOOX_SOURCE}/core/smtcbs.cpp")
    message(FATAL_ERROR "Set -DBOOX_DIR=<boOX checkout>; ${BOOX_SOURCE}/core/smtcbs.cpp was not found")
endif()

find_package(Boost CONFIG QUIET COMPONENTS program_options)
if(NOT Boost_FOUND)
    if(POLICY CMP0167)
        cmake_policy(SET CMP0167 OLD)
    endif()
    set(Boost_NO_BOOST_CMAKE ON)
    find_package(Boost REQUIRED COMPONENTS program_options)
endif()

# OR-Tools is needed only for the verification and fallback paths of the flow code (UA_FLOW_INCREMENTAL=0,
# UA_FLOW_OWN_MINRC=0, UA_FLOW_MIN_COST=1); the default search does not use it.
option(WITH_ORTOOLS "Build the OR-Tools verification and fallback paths" OFF)
if(WITH_ORTOOLS)
    find_package(ortools CONFIG REQUIRED)
endif()

# boOX sources are compiled in this build directory; nothing in the boOX checkout is modified.
add_library(paf_boox STATIC
    ${BOOX_SOURCE}/common/types.cpp
    ${BOOX_SOURCE}/util/statistics.cpp
    ${BOOX_SOURCE}/util/io.cpp
    ${BOOX_SOURCE}/glucose/Options.cc
    ${BOOX_SOURCE}/glucose/Solver.cc
    ${BOOX_SOURCE}/glucose/System.cc
    ${BOOX_SOURCE}/core/graph.cpp
    ${BOOX_SOURCE}/core/agent.cpp
    ${BOOX_SOURCE}/core/cbs.cpp
    ${BOOX_SOURCE}/core/icts.cpp
    ${BOOX_SOURCE}/core/cnf.cpp
    ${BOOX_SOURCE}/core/smtcbs.cpp)
set_target_properties(paf_boox PROPERTIES CXX_STANDARD 14)
target_include_directories(paf_boox SYSTEM PUBLIC
    ${BOOX_SOURCE} ${BOOX_SOURCE}/core ${BOOX_SOURCE}/glucose)

add_library(paf_nrf STATIC src/nrf_backend.cpp)
target_include_directories(paf_nrf PUBLIC include)
target_link_libraries(paf_nrf PRIVATE paf_boox)

add_library(paf_flow STATIC
    src/flow_planner.cpp
    lib/NATCBSDirectEarliestFailMinRC/earliest_fail_minrc_direct_flow_node.cpp
    lib/constraints/constraints.cpp
    lib/constraints/vertex_constraint.cpp
    lib/constraints/edge_constraint.cpp)
target_include_directories(paf_flow PRIVATE src)
target_include_directories(paf_flow SYSTEM PUBLIC lib ${Boost_INCLUDE_DIRS})
target_compile_definitions(paf_flow PUBLIC STRIP_OWN_WITH_ORTOOLS=$<BOOL:${WITH_ORTOOLS}>)
if(WITH_ORTOOLS)
    target_link_libraries(paf_flow PUBLIC ortools::ortools)
endif()

add_library(paf_solver STATIC src/problem.cpp src/hybrid.cpp src/output.cpp)
target_include_directories(paf_solver PUBLIC include PRIVATE src)
target_link_libraries(paf_solver PRIVATE paf_nrf paf_flow)

add_executable(paf_sat src/main.cpp)
target_link_libraries(paf_sat PRIVATE paf_solver Boost::program_options)
foreach(target paf_nrf paf_solver paf_sat)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()
