project(plugin_pb3)

# Collects the names of all the source files in the specified directory
# and stores the list in the <variable> provided
aux_source_directory(${PROJECT_SOURCE_DIR}
        PB_FILES)

add_library(${PROJECT_NAME}
        STATIC ${PB_FILES}) 

# FindProtobuf module
include(FindProtobuf)
find_package(Protobuf REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}
        ${PROTOBUF_INCLUDE_DIR}
        )

link_directories(${PROTOBUF_LIBRARY})
target_link_libraries(${PROJECT_NAME}
        ${PROTOBUF_LIBRARY}
        )
set(CMAKE_CXX_FLAGS "-fPIC")

