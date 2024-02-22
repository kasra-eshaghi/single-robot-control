cmake_minimum_required(VERSION 3.27)
project(single_robot_control)

set(CMAKE_CXX_STANDARD 17)


add_subdirectory(matplotplusplus)




add_executable(single_robot_control main.cpp
        src/Robot.cpp
        include/Robot.h
        src/Controller.cpp
        include/Controller.h
        src/Planning.cpp
        include/Planning.h
        src/Extras.cpp
        include/Extras.h
        src/Localizer.cpp
        include/Localizer.h
        src/Problem_Instance.cpp
        include/Problem_Instance.h
)

target_link_libraries(single_robot_control PUBLIC matplot)

