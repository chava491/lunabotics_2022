# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mars_robot_msgs: 2 messages, 0 services")

set(MSG_I_FLAGS "-Imars_robot_msgs:/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mars_robot_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/sensor_msg.msg" NAME_WE)
add_custom_target(_mars_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mars_robot_msgs" "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/sensor_msg.msg" ""
)

get_filename_component(_filename "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/motor_data_msg.msg" NAME_WE)
add_custom_target(_mars_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mars_robot_msgs" "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/motor_data_msg.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mars_robot_msgs
  "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/sensor_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mars_robot_msgs
)
_generate_msg_cpp(mars_robot_msgs
  "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/motor_data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mars_robot_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(mars_robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mars_robot_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mars_robot_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mars_robot_msgs_generate_messages mars_robot_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/sensor_msg.msg" NAME_WE)
add_dependencies(mars_robot_msgs_generate_messages_cpp _mars_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/motor_data_msg.msg" NAME_WE)
add_dependencies(mars_robot_msgs_generate_messages_cpp _mars_robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mars_robot_msgs_gencpp)
add_dependencies(mars_robot_msgs_gencpp mars_robot_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mars_robot_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(mars_robot_msgs
  "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/sensor_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mars_robot_msgs
)
_generate_msg_eus(mars_robot_msgs
  "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/motor_data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mars_robot_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(mars_robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mars_robot_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mars_robot_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mars_robot_msgs_generate_messages mars_robot_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/sensor_msg.msg" NAME_WE)
add_dependencies(mars_robot_msgs_generate_messages_eus _mars_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/motor_data_msg.msg" NAME_WE)
add_dependencies(mars_robot_msgs_generate_messages_eus _mars_robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mars_robot_msgs_geneus)
add_dependencies(mars_robot_msgs_geneus mars_robot_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mars_robot_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(mars_robot_msgs
  "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/sensor_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mars_robot_msgs
)
_generate_msg_lisp(mars_robot_msgs
  "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/motor_data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mars_robot_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(mars_robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mars_robot_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mars_robot_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mars_robot_msgs_generate_messages mars_robot_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/sensor_msg.msg" NAME_WE)
add_dependencies(mars_robot_msgs_generate_messages_lisp _mars_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/motor_data_msg.msg" NAME_WE)
add_dependencies(mars_robot_msgs_generate_messages_lisp _mars_robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mars_robot_msgs_genlisp)
add_dependencies(mars_robot_msgs_genlisp mars_robot_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mars_robot_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(mars_robot_msgs
  "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/sensor_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mars_robot_msgs
)
_generate_msg_nodejs(mars_robot_msgs
  "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/motor_data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mars_robot_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(mars_robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mars_robot_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mars_robot_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mars_robot_msgs_generate_messages mars_robot_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/sensor_msg.msg" NAME_WE)
add_dependencies(mars_robot_msgs_generate_messages_nodejs _mars_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/motor_data_msg.msg" NAME_WE)
add_dependencies(mars_robot_msgs_generate_messages_nodejs _mars_robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mars_robot_msgs_gennodejs)
add_dependencies(mars_robot_msgs_gennodejs mars_robot_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mars_robot_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mars_robot_msgs
  "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/sensor_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mars_robot_msgs
)
_generate_msg_py(mars_robot_msgs
  "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/motor_data_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mars_robot_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(mars_robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mars_robot_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mars_robot_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mars_robot_msgs_generate_messages mars_robot_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/sensor_msg.msg" NAME_WE)
add_dependencies(mars_robot_msgs_generate_messages_py _mars_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mars/Documents/GitHub/MARS-Lunabotics-2022/catkin_ws/src/mars_robot_msgs/msg/motor_data_msg.msg" NAME_WE)
add_dependencies(mars_robot_msgs_generate_messages_py _mars_robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mars_robot_msgs_genpy)
add_dependencies(mars_robot_msgs_genpy mars_robot_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mars_robot_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mars_robot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mars_robot_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(mars_robot_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mars_robot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mars_robot_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(mars_robot_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mars_robot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mars_robot_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(mars_robot_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mars_robot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mars_robot_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(mars_robot_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mars_robot_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mars_robot_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mars_robot_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(mars_robot_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
