cmake_minimum_required(VERSION 3.10)

#find_package(Libigl CONFIG REQUIRED)

project(laplacian VERSION 1.0)

set(LXSDK_PATH "../LXSDK-000025")

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
    	$<$<PLATFORM_ID:Darwin>:/usr/local/include>
)

target_compile_options(laplacian
    PRIVATE
        $<$<CXX_COMPILER_ID:AppleClang>:
            -Wno-deprecated-declarations
            -Wno-error
        >
)

target_compile_definitions(laplacian
    PRIVATE
        IGL_STATIC_LIBRARY
)

# Set your Igl libraray
target_link_libraries(laplacian
    PUBLIC
	    lxsdk
    PRIVATE
   	    $<$<PLATFORM_ID:Darwin>:/usr/local/lib/libigl.a>
#        Libigl::Libigl
)
