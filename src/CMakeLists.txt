
#bilateralteleop

# header-only
set(utils_HDR_DIR
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/bilateralteleop>$<INSTALL_INTERFACE:include/bilateralteleop>)

set(utils_HDR
  ${utils_HDR_DIR}/HumanRobotPose.h
)

# header-only
add_library(utils_${PROJECT_NAME} INTERFACE)
if(COMMAND target_sources)
  target_sources(utils_${PROJECT_NAME} INTERFACE ${utils_HDR})
endif()
add_library(bilateralteleop_utils ALIAS utils_${PROJECT_NAME})

install(TARGETS utils_${PROJECT_NAME} 
  EXPORT "${TARGETS_EXPORT_NAME}"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

#Tasks

set(Tasks_SRC
  Tasks/biRobotTeleopTask.cpp
)

set(Tasks_HDR
  ../include/Tasks/biRobotTeleopTask.h
)

add_library(Tasks_${PROJECT_NAME} SHARED ${Tasks_SRC} ${Tasks_HDR})

target_link_libraries(Tasks_${PROJECT_NAME}
                      PUBLIC 
                      SpaceVecAlg::SpaceVecAlg 
                      ${Boost_LIBRARIES}
                      eigen-quadprog
                      Tasks
                      )

set_target_properties(Tasks_${PROJECT_NAME} PROPERTIES
  OUTPUT_NAME Tasks_${PROJECT_NAME}
  VERSION Tasks_${PROJECT_NAME}
  PUBLIC_HEADER "${Tasks_${PROJECT_NAME}_HDR}")

target_include_directories(Tasks_${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/Tasks>"
  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

install(TARGETS Tasks_${PROJECT_NAME}
  EXPORT "${TARGETS_EXPORT_NAME}"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)


#mc_tasks

set(mc_tasks_SRC
  mc_tasks/biRobotTeleopTask.cpp
)

set(mc_tasks_HDR
  ../include/mc_tasks/biRobotTeleopTask.h
)

add_library(mc_tasks_${PROJECT_NAME} SHARED ${mc_tasks_SRC} ${mc_tasks_HDR})

target_link_libraries(mc_tasks_${PROJECT_NAME}
                      PUBLIC 
                      SpaceVecAlg::SpaceVecAlg 
                      ${Boost_LIBRARIES}
                      eigen-quadprog)
target_link_libraries(mc_tasks_${PROJECT_NAME} PUBLIC utils_${PROJECT_NAME})
target_link_libraries(mc_tasks_${PROJECT_NAME} PUBLIC Tasks_${PROJECT_NAME})
target_link_libraries(mc_tasks_${PROJECT_NAME} 
                      PUBLIC
                      mc_tasks)


set_target_properties(mc_tasks_${PROJECT_NAME} PROPERTIES
  OUTPUT_NAME mc_tasks_${PROJECT_NAME}
  VERSION mc_tasks_${PROJECT_NAME}
  PUBLIC_HEADER "${mc_tasks_${PROJECT_NAME}_HDR}")

target_include_directories(mc_tasks_${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/mc_task>"
  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

install(TARGETS mc_tasks_${PROJECT_NAME}
  EXPORT "${TARGETS_EXPORT_NAME}"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(
DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)

