
# /***************************************************************************
# *  CMakelist.txt - CMake Script for Protobuf Client
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
project(refbox_node_example)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation sensor_msgs refbox_msgs genmsg roslaunch)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(OpenSSL REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -DHAVE_LIBCRYPTO")

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS refbox_msgs)

add_executable(refbox_node_example
   src/client.cpp
   src/crypto.cpp
   src/message_register.cpp
   src/peer.cpp
   src/server.cpp
   src/refbox_node_example.cpp
)

target_link_libraries(refbox_node_example
   ${catkin_LIBRARIES}
   ${OPENSSL_LIBRARIES}
   ${Boost_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
