cmake_minimum_required(VERSION 3.12)

set(INCLUDE_DIRS)
function(add_include_dir IDENTIFIER)
	set(PRETTYNAME ${IDENTIFIER})
	set(ARGV ${ARGN})
	list(LENGTH ARGV ARGC)
	if(${ARGC} GREATER 0)
		list(GET ARGV 0 PRETTYNAME)
	endif()
	string(TOUPPER ${IDENTIFIER} UIDENTIFIER)

	set(${DEFAULT_DIR} "")
	set(DEPENDENCY_${UIDENTIFIER}_INCLUDE ${DEFAULT_DIR} CACHE PATH "Path to ${PRETTYNAME} include directory.")
	set(INCLUDE_DIRS ${INCLUDE_DIRS} DEPENDENCY_${UIDENTIFIER}_INCLUDE PARENT_SCOPE)
endfunction(add_include_dir)

set(LIBRARIES)
function(add_external_library IDENTIFIER)
	set(PRETTYNAME ${IDENTIFIER})
	set(ARGV ${ARGN})
	list(LENGTH ARGV ARGC)
	if(${ARGC} GREATER 0)
		list(GET ARGV 0 PRETTYNAME)
	endif()
	string(TOUPPER ${IDENTIFIER} UIDENTIFIER)

	set(DEPENDENCY_${UIDENTIFIER}_LIBRARY "" CACHE FILEPATH "Path to ${PRETTYNAME} library.")
	set(LIBRARIES ${LIBRARIES} DEPENDENCY_${UIDENTIFIER}_LIBRARY PARENT_SCOPE)
endfunction(add_external_library)

function(link_external_library IDENTIFIER)
	set(PRETTYNAME ${IDENTIFIER})
	set(ARGV ${ARGN})
	list(LENGTH ARGV ARGC)
	if(${ARGC} GREATER 0)
		list(GET ARGV 0 PRETTYNAME)
	endif()
	string(TOUPPER ${IDENTIFIER} UIDENTIFIER)

	set(${DEFAULT_DIR} "")
	set(DEPENDENCY_${UIDENTIFIER}_INCLUDE ${DEFAULT_DIR} CACHE PATH "Path to ${PRETTYNAME} include directory.")
	set(INCLUDE_DIRS ${INCLUDE_DIRS} DEPENDENCY_${UIDENTIFIER}_INCLUDE PARENT_SCOPE)

	set(DEPENDENCY_${UIDENTIFIER}_LIBRARY "" CACHE FILEPATH "Path to ${PRETTYNAME} library.")
	set(LIBRARIES ${LIBRARIES} DEPENDENCY_${UIDENTIFIER}_LIBRARY PARENT_SCOPE)
endfunction(link_external_library)

set(DEFINITIONS)
function(add_def DEF)
	set(DEFINITIONS ${DEFINITIONS} ${DEF} PARENT_SCOPE)
endfunction(add_def)

##### CONFIGURATION #####

set(PROJ_NAME UniRender_LuxCoreRender)

project(${PROJ_NAME} CXX)

set(CMAKE_CXX_STANDARD 20)
set(ENABLE_STATIC_LIBRARY_FLAG 0)

link_external_library(util_image)
link_external_library(util_raytracing)
link_external_library(sharedutils)
link_external_library(mathutil)
link_external_library(vfilesystem)
link_external_library(luxcorerender)
add_include_dir(luxcorerender_generated)

set(DEPENDENCY_LIBBOOST_FILESYSTEM_LIBRARY "${DEPENDENCY_BOOST_LIBRARY_LOCATION}/libboost_filesystem-vc142-mt-x64-1_76.lib" CACHE PATH "Path to library." FORCE)
set(DEPENDENCY_LIBBOOST_IOSTREAMS_LIBRARY "${DEPENDENCY_BOOST_LIBRARY_LOCATION}/libboost_iostreams-vc142-mt-x64-1_76.lib" CACHE PATH "Path to library." FORCE)
set(DEPENDENCY_LIBBOOST_SERIALIZATION_LIBRARY "${DEPENDENCY_BOOST_LIBRARY_LOCATION}/libboost_serialization-vc142-mt-x64-1_76.lib" CACHE PATH "Path to library." FORCE)
set(DEPENDENCY_LIBBOOST_ZLIB_LIBRARY "${DEPENDENCY_BOOST_LIBRARY_LOCATION}/libboost_zlib-vc142-mt-x64-1_76.lib" CACHE PATH "Path to library." FORCE)
set(DEPENDENCY_LIBBOOST_THREAD_LIBRARY "${DEPENDENCY_BOOST_LIBRARY_LOCATION}/libboost_thread-vc142-mt-x64-1_76.lib" CACHE PATH "Path to library." FORCE)
set(DEPENDENCY_LIBBOOST_REGEX_LIBRARY "${DEPENDENCY_BOOST_LIBRARY_LOCATION}/libboost_regex-vc142-mt-x64-1_76.lib" CACHE PATH "Path to library." FORCE)

set(DEPENDENCY_LUXCORERENDER_LIBRARY_LOCATION "" CACHE PATH "Path to LuxCoreRender lib directory.")
set(DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION "" CACHE PATH "Path to LuxCoreRender lib directory.")
set(DEPENDENCY_LUXCORERENDER_OPENSUBDIV_LIBRARY "${DEPENDENCY_LUXCORERENDER_LIBRARY_LOCATION}/opensubdiv.lib" CACHE PATH "Path to library." FORCE)
# set(DEPENDENCY_LUXCORERENDER_OPENIMAGEIO_LIBRARY "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/OpenImageIO.lib" CACHE PATH "Path to library." FORCE)
# set(DEPENDENCY_LUXCORERENDER_OPENIMAGEDENOISE_LIBRARY "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/OpenImageDenoise.lib" CACHE PATH "Path to library." FORCE)
set(DEPENDENCY_LUXCORERENDER_TIFF_LIBRARY "${DEPENDENCY_TIFF_LIBRARY}" CACHE PATH "Path to library." FORCE)
set(DEPENDENCY_LUXCORERENDER_LPNG_LIBRARY "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/libpng.lib" CACHE PATH "Path to library." FORCE)
set(DEPENDENCY_LUXCORERENDER_ZLIB_LIBRARY "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/zlib1.lib" CACHE PATH "Path to library." FORCE)
set(DEPENDENCY_LUXCORERENDER_JPEG_LIBRARY "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/jpeg.lib" CACHE PATH "Path to library." FORCE)
# set(DEPENDENCY_LUXCORERENDER_OPENEXR_HALF_LIBRARY "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/Half.lib" CACHE PATH "Path to library." FORCE)
# set(DEPENDENCY_LUXCORERENDER_OPENEXR_IEX_LIBRARY "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/Iex.lib" CACHE PATH "Path to library." FORCE)
# set(DEPENDENCY_LUXCORERENDER_OPENEXR_ILMTHREAD_LIBRARY "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/IlmThread.lib" CACHE PATH "Path to library." FORCE)
# set(DEPENDENCY_LUXCORERENDER_OPENEXR_IMATH_LIBRARY "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/Imath.lib" CACHE PATH "Path to library." FORCE)
# set(DEPENDENCY_LUXCORERENDER_OPENEXR_ILMIMF_LIBRARY "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/IlmImf.lib" CACHE PATH "Path to library." FORCE)
# set(DEPENDENCY_LUXCORERENDER_OPENIMAGEIO_LIBRARY "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/OpenImageIO.lib" CACHE PATH "Path to library." FORCE)
set(DEPENDENCY_LUXCORERENDER_BCD_LIBRARY "${DEPENDENCY_LUXCORERENDER_LIBRARY_LOCATION}/bcd.lib" CACHE PATH "Path to library." FORCE)
set(DEPENDENCY_LUXCORERENDER_BLOSC_LIBRARY "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/blosc.lib" CACHE PATH "Path to library." FORCE)
set(DEPENDENCY_LUXCORERENDER_EMBREE3_LIBRARY "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/embree3.lib" CACHE PATH "Path to library." FORCE)
set(DEPENDENCY_LUXCORERENDER_OPENVDB_LIBRARY "${DEPENDENCY_LUXCORERENDER_LIBRARY_LOCATION}/openvdb.lib" CACHE PATH "Path to library." FORCE)
set(DEPENDENCY_LUXCORERENDER_LUXRAYS_LIBRARY "${DEPENDENCY_LUXCORERENDER_LIBRARY_LOCATION}/luxrays.lib" CACHE PATH "Path to library." FORCE)
set(DEPENDENCY_LUXCORERENDER_SLGCORE_LIBRARY "${DEPENDENCY_LUXCORERENDER_LIBRARY_LOCATION}/slg-core.lib" CACHE PATH "Path to library." FORCE)
set(DEPENDENCY_LUXCORERENDER_SLGFILM_LIBRARY "${DEPENDENCY_LUXCORERENDER_LIBRARY_LOCATION}/slg-film.lib" CACHE PATH "Path to library." FORCE)
set(DEPENDENCY_LUXCORERENDER_SLGKERNELS_LIBRARY "${DEPENDENCY_LUXCORERENDER_LIBRARY_LOCATION}/slg-kernels.lib" CACHE PATH "Path to library." FORCE)
set(DEPENDENCY_LUXCORERENDER_TBB_LIBRARY "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/tbb.lib" CACHE PATH "Path to library." FORCE)

add_include_dir(openimageio)
add_external_library(luxcorerender_openimageio)
add_external_library(luxcorerender_openimageio_util)
link_external_library(luxcorerender_openimagedenoise)
link_external_library(util_udm)
add_include_dir(glm)
add_include_dir(boost)
add_external_library(glew)
add_external_library(luxcorerender_tiff)
add_external_library(luxcorerender_lpng)
add_external_library(luxcorerender_zlib)
add_external_library(luxcorerender_jpeg)

# Required for color management
link_external_library(util_ocio)
add_external_library(luxcorerender_OpenEXR_Half)
add_external_library(luxcorerender_OpenEXR_Iex)
add_external_library(luxcorerender_OpenEXR_IlmThread)
add_external_library(luxcorerender_OpenEXR_Imath)
add_external_library(luxcorerender_OpenEXR_IlmImf)
add_external_library(opencolorio)
add_external_library(luxcorerender_OpenImageIO)
add_external_library(luxcorerender_OpenImageIO_util)
#

add_external_library(libboost_thread)
add_external_library(libboost_regex)
add_external_library(libboost_filesystem)
add_external_library(libboost_iostreams)
add_external_library(libboost_zlib)
add_external_library(libboost_serialization)
add_external_library(luxcorerender_opensubdiv)
add_external_library(luxcorerender_bcd)
add_external_library(luxcorerender_blosc)
add_external_library(luxcorerender_embree3)
add_external_library(luxcorerender_openvdb)
add_external_library(luxcorerender_luxrays)
add_external_library(luxcorerender_slgcore)
add_external_library(luxcorerender_slgfilm)
add_external_library(luxcorerender_slgkernels)
add_external_library(luxcorerender_tbb)

add_def(OIIO_STATIC_BUILD)
add_def(GLEW_STATIC)
add_def(RTUTIL_DLL)
add_def(UIMG_ENABLE_NVTT)

##### CONFIGURATION #####

set(LIB_TYPE SHARED)

foreach(def IN LISTS DEFINITIONS)
	add_definitions(-D${def})
endforeach(def)

function(def_vs_filters FILE_LIST)
	foreach(source IN LISTS FILE_LIST)
	    get_filename_component(source_path "${source}" PATH)
	    string(REPLACE "${CMAKE_CURRENT_LIST_DIR}" "" source_path_relative "${source_path}")
	    string(REPLACE "/" "\\" source_path_msvc "${source_path_relative}")
	    source_group("${source_path_msvc}" FILES "${source}")
	endforeach()
endfunction(def_vs_filters)

file(GLOB_RECURSE SRC_FILES
    "${CMAKE_CURRENT_LIST_DIR}/src/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/src/*.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/include/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/*.hpp"
)
add_library(${PROJ_NAME} ${LIB_TYPE} ${SRC_FILES})
if(WIN32)
	target_compile_options(${PROJ_NAME} PRIVATE /wd4251)
	target_compile_options(${PROJ_NAME} PRIVATE /wd4996)
endif()
def_vs_filters("${SRC_FILES}")

foreach(LIB IN LISTS LIBRARIES)
	target_link_libraries(${PROJ_NAME} ${${LIB}})
endforeach(LIB)

target_include_directories(${PROJ_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(${PROJ_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

foreach(INCLUDE_PATH IN LISTS INCLUDE_DIRS)
	target_include_directories(${PROJ_NAME} PRIVATE ${${INCLUDE_PATH}})
endforeach(INCLUDE_PATH)

set(TARGET_PROPERTIES LINKER_LANGUAGE CXX)
if(DEFINED PRECOMPILED_HEADER)
	include(PrecompiledHeader.cmake)
	add_precompiled_header(${PROJ_NAME} "src/${PRECOMPILED_HEADER}.h" c++17 FORCEINCLUDE)
endif()
set_target_properties(${PROJ_NAME} PROPERTIES ${TARGET_PROPERTIES})

target_link_libraries(${PROJ_NAME}
	opengl32
	glu32
	bcrypt
)

add_dependencies(${PROJ_NAME} util_raytracing)
