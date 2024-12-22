cmake_minimum_required(VERSION 3.10)

#find_package(Libigl CONFIG REQUIRED)

project(laplacian VERSION 1.0)
include(FetchContent)

# set your directory pathes below
#set(LXSDK_PATH "../LXSDK-000025")
#set(LIBIGL_PATH "../libigl-2.5.0")
#set(EIGEN_PATH "../eigen-3.4.0")

if (NOT LXSDK_PATH)
    FetchContent_Declare( 
        lxsdk
        URL https://thefoundry.s3.us-east-1.amazonaws.com/products/modo/16.1v5/lxsdk16.1v5_692025.zip
    )
    FetchContent_Populate(lxsdk)
    message(STATUS "FetchContent lxsdk directory: ${lxsdk_SOURCE_DIR}")
    set(LXSDK_PATH ${lxsdk_SOURCE_DIR})
endif()

# Get all source and headers for lxsdk
file(GLOB LXSDK_SOURCES ${LXSDK_PATH}/common/*.cpp)
file(GLOB LXSDK_HEADERS ${LXSDK_PATH}/include/*.h?)

add_library(lxsdk
    STATIC
        ${LXSDK_SOURCES}
)

set_target_properties(lxsdk 
    PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY lib
        POSITION_INDEPENDENT_CODE ON
)

target_compile_definitions(lxsdk
    PRIVATE
        GL_SILENCE_DEPRECATION
)

target_compile_features(lxsdk
    PRIVATE
        cxx_std_17
)

target_include_directories(lxsdk
    PRIVATE
        ${LXSDK_PATH}/include
)


if (NOT EIGEN_PATH)
    FetchContent_Declare( 
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG        3.4.0
    )
    FetchContent_Populate(eigen)
    message(STATUS "FetchContent eigen directory: ${eigen_SOURCE_DIR}")
    set(EIGEN_PATH ${eigen_SOURCE_DIR})
endif()

if (NOT LIBIGL_PATH)
    FetchContent_Declare( 
        libigl
        GIT_REPOSITORY https://github.com/libigl/libigl.git
        GIT_TAG        v2.5.0
    )
    FetchContent_Populate(libigl)
    message(STATUS "FetchContent libigl directory: ${libigl_SOURCE_DIR}")
    set(LIBIGL_PATH ${libigl_SOURCE_DIR})
endif()

add_library(laplacian
    SHARED
        "source/mesh_helper.cxx"
        "source/tool.cxx"
)

target_compile_features(laplacian
    PRIVATE
        cxx_std_17
)

# Set your LXSDK include directry
target_include_directories(laplacian 
    PUBLIC
        ${LXSDK_PATH}/include
    	${LIBIGL_PATH}/include
    	${EIGEN_PATH}
)

target_compile_options(laplacian
    PRIVATE
        $<$<CXX_COMPILER_ID:AppleClang>:
            -Wno-deprecated-declarations
            -Wno-error
        >
        $<$<CXX_COMPILER_ID:MSVC>:
            /bigobj
        >
)

# Set your Igl libraray
target_link_libraries(laplacian
    PUBLIC
	    lxsdk
)

# Install plugins and configs into kit directry
if (NOT CMAKE_INSTALL_PREFIX)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/Laplacian")
endif()
install(
    FILES
        "index.cfg"
        "index.html"
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
)
install(
    DIRECTORY
        "images"
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
)
install(
    TARGETS
        laplacian
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}/extra
)
