# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "local_planner: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ilocal_planner:/home/panthera/panthera_ws/src/local_planner/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(local_planner_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/panthera/panthera_ws/src/local_planner/msg/CmapClear.msg" NAME_WE)
add_custom_target(_local_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "local_planner" "/home/panthera/panthera_ws/src/local_planner/msg/CmapClear.msg" ""
)

get_filename_component(_filename "/home/panthera/panthera_ws/src/local_planner/msg/Sonar.msg" NAME_WE)
add_custom_target(_local_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "local_planner" "/home/panthera/panthera_ws/src/local_planner/msg/Sonar.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(local_planner
  "/home/panthera/panthera_ws/src/local_planner/msg/CmapClear.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/local_planner
)
_generate_msg_cpp(local_planner
  "/home/panthera/panthera_ws/src/local_planner/msg/Sonar.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/local_planner
)

### Generating Services

### Generating Module File
_generate_module_cpp(local_planner
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/local_planner
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(local_planner_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(local_planner_generate_messages local_planner_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/panthera/panthera_ws/src/local_planner/msg/CmapClear.msg" NAME_WE)
add_dependencies(local_planner_generate_messages_cpp _local_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/panthera/panthera_ws/src/local_planner/msg/Sonar.msg" NAME_WE)
add_dependencies(local_planner_generate_messages_cpp _local_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(local_planner_gencpp)
add_dependencies(local_planner_gencpp local_planner_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS local_planner_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(local_planner
  "/home/panthera/panthera_ws/src/local_planner/msg/CmapClear.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/local_planner
)
_generate_msg_eus(local_planner
  "/home/panthera/panthera_ws/src/local_planner/msg/Sonar.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/local_planner
)

### Generating Services

### Generating Module File
_generate_module_eus(local_planner
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/local_planner
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(local_planner_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(local_planner_generate_messages local_planner_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/panthera/panthera_ws/src/local_planner/msg/CmapClear.msg" NAME_WE)
add_dependencies(local_planner_generate_messages_eus _local_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/panthera/panthera_ws/src/local_planner/msg/Sonar.msg" NAME_WE)
add_dependencies(local_planner_generate_messages_eus _local_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(local_planner_geneus)
add_dependencies(local_planner_geneus local_planner_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS local_planner_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(local_planner
  "/home/panthera/panthera_ws/src/local_planner/msg/CmapClear.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/local_planner
)
_generate_msg_lisp(local_planner
  "/home/panthera/panthera_ws/src/local_planner/msg/Sonar.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/local_planner
)

### Generating Services

### Generating Module File
_generate_module_lisp(local_planner
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/local_planner
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(local_planner_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(local_planner_generate_messages local_planner_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/panthera/panthera_ws/src/local_planner/msg/CmapClear.msg" NAME_WE)
add_dependencies(local_planner_generate_messages_lisp _local_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/panthera/panthera_ws/src/local_planner/msg/Sonar.msg" NAME_WE)
add_dependencies(local_planner_generate_messages_lisp _local_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(local_planner_genlisp)
add_dependencies(local_planner_genlisp local_planner_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS local_planner_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(local_planner
  "/home/panthera/panthera_ws/src/local_planner/msg/CmapClear.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/local_planner
)
_generate_msg_nodejs(local_planner
  "/home/panthera/panthera_ws/src/local_planner/msg/Sonar.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/local_planner
)

### Generating Services

### Generating Module File
_generate_module_nodejs(local_planner
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/local_planner
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(local_planner_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(local_planner_generate_messages local_planner_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/panthera/panthera_ws/src/local_planner/msg/CmapClear.msg" NAME_WE)
add_dependencies(local_planner_generate_messages_nodejs _local_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/panthera/panthera_ws/src/local_planner/msg/Sonar.msg" NAME_WE)
add_dependencies(local_planner_generate_messages_nodejs _local_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(local_planner_gennodejs)
add_dependencies(local_planner_gennodejs local_planner_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS local_planner_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(local_planner
  "/home/panthera/panthera_ws/src/local_planner/msg/CmapClear.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/local_planner
)
_generate_msg_py(local_planner
  "/home/panthera/panthera_ws/src/local_planner/msg/Sonar.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/local_planner
)

### Generating Services

### Generating Module File
_generate_module_py(local_planner
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/local_planner
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(local_planner_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(local_planner_generate_messages local_planner_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/panthera/panthera_ws/src/local_planner/msg/CmapClear.msg" NAME_WE)
add_dependencies(local_planner_generate_messages_py _local_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/panthera/panthera_ws/src/local_planner/msg/Sonar.msg" NAME_WE)
add_dependencies(local_planner_generate_messages_py _local_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(local_planner_genpy)
add_dependencies(local_planner_genpy local_planner_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS local_planner_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/local_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/local_planner
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(local_planner_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(local_planner_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(local_planner_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/local_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/local_planner
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(local_planner_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(local_planner_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(local_planner_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/local_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/local_planner
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(local_planner_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(local_planner_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(local_planner_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/local_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/local_planner
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(local_planner_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(local_planner_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(local_planner_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/local_planner)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/local_planner\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/local_planner
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(local_planner_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(local_planner_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(local_planner_generate_messages_py std_msgs_generate_messages_py)
endif()
