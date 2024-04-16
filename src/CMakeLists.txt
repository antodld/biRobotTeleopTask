
#biRobotTeleop

set(utils_HDR
  ../include/biRobotTeleop/motion.h
  ../include/biRobotTeleop/transformation.h
  ../include/biRobotTeleop/type.h
  ../include/biRobotTeleop/HumanRobotPose.h
  ../include/biRobotTeleop/HumanRobotDataReceiver.h
)

set(utils_SRC
    biRobotTeleop/HumanRobotPose.cpp
    biRobotTeleop/HumanRobotDataReceiver.cpp
)

add_library(utils_${PROJECT_NAME} SHARED ${utils_SRC} ${utils_HDR})

target_link_libraries(utils_${PROJECT_NAME}
                      PUBLIC 
                      SpaceVecAlg::SpaceVecAlg 
                      Boost::serialization
                      mc_rtc::mc_control
                      mc_rtc::mc_rtc_gui
                      mc_rtc::mc_control_client
                      )

set_target_properties(utils_${PROJECT_NAME} PROPERTIES
  OUTPUT_NAME utils_${PROJECT_NAME}
  VERSION utils_${PROJECT_NAME}
  PUBLIC_HEADER "${utils_${PROJECT_NAME}_HDR}")

target_include_directories(utils_${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/biRobotTeleop>"
  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

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
                      eigen-quadprog::eigen-quadprog
                      Tasks::Tasks
                      )

set_target_properties(Tasks_${PROJECT_NAME} PROPERTIES
  OUTPUT_NAME Tasks_${PROJECT_NAME}
  VERSION Tasks_${PROJECT_NAME}
  PUBLIC_HEADER "${Tasks_${PROJECT_NAME}_HDR}"
  )

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
                      eigen-quadprog::eigen-quadprog)
target_link_libraries(mc_tasks_${PROJECT_NAME} PUBLIC utils_${PROJECT_NAME})
target_link_libraries(mc_tasks_${PROJECT_NAME} PUBLIC Tasks_${PROJECT_NAME})
target_link_libraries(mc_tasks_${PROJECT_NAME} 
                      PUBLIC
                      mc_rtc::mc_tasks)


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

