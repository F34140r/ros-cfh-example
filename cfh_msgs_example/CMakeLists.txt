
# /***************************************************************************
# *  CMakelist.txt - CMake Script for Protobuf Messages
# *
# *  Created: Sa Mar 21 00:19:17 2015
# *  Copyright  2015 Kai Seidensticker
# ****************************************************************************/
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required(VERSION 2.8.3)
project(cfh_msgs_example)

find_package(catkin REQUIRED message_generation genmsg std_msgs)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)
find_package(ProtocolBuffers REQUIRED)

set(proto_dir ${PROJECT_SOURCE_DIR}/protos)
file(GLOB proto_files "${proto_dir}/*.proto")
message(STATUS "Proto Source Dir: ${proto_dir}")
message(STATUS "Proto Source Files: ${proto_files}")

# Set up destination directories
catkin_destinations()
set(proto_gen_cpp_dir ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})
file(MAKE_DIRECTORY ${proto_gen_cpp_dir}
)

set(protogen_include_dirs ${proto_gen_cpp_dir})
message(STATUS "Proto Include Dirs: ${protogen_include_dirs}")

add_message_files(FILES
  AttentionMessage.msg
  CameraControl.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS ${protogen_include_dirs}
  LIBRARIES ${PROJECT_NAME}_proto
)

# Create lists of files to be generated.
set(proto_gen_cpp_files "")

foreach(proto_file ${proto_files})
  get_filename_component(proto_name ${proto_file} NAME_WE)
  list(APPEND proto_gen_cpp_files ${proto_gen_cpp_dir}/${proto_name}.pb.h ${proto_gen_cpp_dir}/${proto_name}.pb.cc)
endforeach(proto_file ${proto_files})

# Run protoc and generate language-specific headers.
add_custom_command(OUTPUT ${proto_gen_cpp_files}
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} --proto_path=${proto_dir} --cpp_out=${proto_gen_cpp_dir} ${proto_files}
    DEPENDS ${PROTOBUF_PROTOC_EXECUTABLE} ${proto_files}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

set_source_files_properties(${proto_gen_cpp_files} PROPERTIES GENERATED TRUE)

add_custom_target(${PROJECT_NAME}_generate_headers
    DEPENDS ${proto_gen_cpp_files}
)

# Create proto library for linking.
include_directories(${PROTOBUF_INCLUDE_DIR} ${PROTOBUF_INCLUDE_DIR}/../../)
add_library(${PROJECT_NAME}_proto ${proto_gen_cpp_files})
target_link_libraries(${PROJECT_NAME}_proto ${PROTOBUF_LIBRARY})
add_dependencies(${PROJECT_NAME}_proto ${PROJECT_NAME}_generate_headers)

install(TARGETS ${PROJECT_NAME}_proto
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${proto_gen_cpp_dir}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
