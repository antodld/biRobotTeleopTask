cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

set(PROJECT_NAME biRobotTeleop)
set(PROJECT_VERSION 0.1.0)
set(PROJECT_DESCRIPTION "")
set(PROJECT_URL "")

# jrl-cmakemodules minimal settings
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CXX_DISABLE_WERROR 1) # FIXME fix errors and re-enable

include(cmake/base.cmake)

project(${PROJECT_NAME} LANGUAGES CXX VERSION ${PROJECT_VERSION})

add_project_dependency(mc_rtc REQUIRED)
add_project_dependency(RBDyn REQUIRED NO_MODULE)
add_project_dependency(sch-core REQUIRED NO_MODULE)
add_project_dependency(eigen-qld REQUIRED NO_MODULE)
add_project_dependency(Boost REQUIRED COMPONENTS filesystem timer ${BOOST_STACKTRACE})


find_package(SpaceVecAlg REQUIRED)
find_package(mc_rtc REQUIRED)

add_subdirectory(src)