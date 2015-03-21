# Find Protobuf and set useful variables
message(STATUS "Protobuf Include Path Hints: ${PROTOBUF_INCLUDE_PATH_HINTS}")
FIND_PATH(PROTOBUF_INCLUDE_DIR stubs/common.h
  HINTS ${PROTOBUF_INCLUDE_PATH_HINTS}
  PATHS /usr/include/google/protobuf
)
message(STATUS "PROTOBUF_INCLUDE_DIR: ${PROTOBUF_INCLUDE_DIR}")

message(STATUS "Protobuf Lib Path Hints: ${PROTOBUF_LIB_PATH_HINTS}")
FIND_LIBRARY(PROTOBUF_LIBRARY protobuf
  HINTS ${PROTOBUF_LIB_PATH_HINTS}
)
message(STATUS "PROTOBUF_LIBRARY: ${PROTOBUF_LIBRARY}")

message(STATUS "Protobuf Exe Path Hints: ${PROTOBUF_EXE_PATH_HINTS}")
FIND_PROGRAM(PROTOBUF_PROTOC_EXECUTABLE protoc
  HINTS ${PROTOBUF_EXE_PATH_HINTS}
)
message(STATUS "PROTOBUF_PROTOC_EXECUTABLE: ${PROTOBUF_PROTOC_EXECUTABLE}")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(protobuf DEFAULT_MSG PROTOBUF_INCLUDE_DIR PROTOBUF_LIBRARY PROTOBUF_PROTOC_EXECUTABLE)
message(STATUS "PROTOBUF_FOUND: ${PROTOBUF_FOUND}")

# ensure that they are cached
SET(PROTOBUF_INCLUDE_DIR ${PROTOBUF_INCLUDE_DIR} CACHE INTERNAL "The protocol buffers include path")
SET(PROTOBUF_LIBRARY ${PROTOBUF_LIBRARY} CACHE INTERNAL "The libraries needed to use protocol buffers library")
SET(PROTOBUF_PROTOC_EXECUTABLE ${PROTOBUF_PROTOC_EXECUTABLE} CACHE INTERNAL "The protocol buffers compiler")


function(get_protogen_include_dir)
    set(proto_dir ${PROJECT_SOURCE_DIR}/proto)
    set(proto_gen_dir ${CMAKE_CURRENT_BINARY_DIR}/proto_gen)
    set(proto_gen_cpp_dir ${proto_gen_dir}/cpp/include/${PROJECT_NAME})
    set(proto_gen_python_dir ${proto_gen_dir}/python)
    file(MAKE_DIRECTORY ${proto_gen_dir})
    file(MAKE_DIRECTORY ${proto_gen_cpp_dir})
    file(MAKE_DIRECTORY ${proto_gen_python_dir})
    set(PROTOGEN_INCLUDE_DIR ${proto_gen_cpp_dir}/../ ${proto_gen_python_dir})
endfunction()

function(run_protogen PROTO_FILES)
    # Set up some base variables.
    set(proto_dir ${PROJECT_SOURCE_DIR}/proto)
    message(STATUS "Proto Dir: ${proto_dir}")
    message(STATUS "Proto Files: ${PROTO_FILES}")
    set(proto_gen_dir ${CMAKE_CURRENT_BINARY_DIR}/proto_gen)
    set(proto_gen_cpp_dir ${proto_gen_dir}/cpp/include/${PROJECT_NAME})
    set(proto_gen_python_dir ${proto_gen_dir}/python)
    file(MAKE_DIRECTORY ${proto_gen_dir})
    file(MAKE_DIRECTORY ${proto_gen_cpp_dir})
    file(MAKE_DIRECTORY ${proto_gen_python_dir})
    set(PROTOGEN_INCLUDE_DIR ${proto_gen_cpp_dir}/../ ${proto_gen_python_dir})

    # Create lists of files to be generated.
    set(proto_gen_cpp_files "")
    set(proto_gen_python_files "")
    foreach(proto_file ${PROTO_FILES})
        get_filename_component(proto_name ${proto_file} NAME_WE)
        list(APPEND proto_gen_cpp_files ${proto_gen_cpp_dir}/${proto_name}.pb.h ${proto_gen_cpp_dir}/${proto_name}.pb.cc)
        list(APPEND proto_gen_python_files ${proto_gen_python_dir}/${proto_name}_pb2.py)
    endforeach(proto_file ${PROTO_FILES})

    # Run protoc and generate language-specific headers.
    add_custom_command(
        COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} --proto_path=${proto_dir} --cpp_out=${proto_gen_cpp_dir} --python_out=${proto_gen_python_dir} ${PROTO_FILES}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DEPENDS ${PROTOBUF_PROTOC_EXECUTABLE} ${PROTO_FILES}
        OUTPUT ${proto_gen_cpp_files} ${proto_gen_python_files}
    )

    # Create single proto library for linking.
    include_directories(${PROTOBUF_INCLUDE_DIR} ${PROTOBUF_INCLUDE_DIR}/../../)
    add_library(${PROJECT_NAME}_proto ${proto_gen_cpp_files})
    target_link_libraries(${PROJECT_NAME}_proto ${PROTOBUF_LIBRARY})

    install(TARGETS ${PROJECT_NAME}_proto
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    install(DIRECTORY ${proto_gen_cpp_dir}/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      FILES_MATCHING PATTERN "*.h"
    )

    install(DIRECTORY ${proto_gen_python_dir}/
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
      FILES_MATCHING PATTERN "*.py"
    )

    SET(PROTOGEN_INCLUDE_DIR ${PROTOGEN_INCLUDE_DIR} CACHE INTERNAL "The generated headers include path")
    endfunction()
