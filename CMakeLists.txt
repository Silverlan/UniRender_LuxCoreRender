include(${CMAKE_SOURCE_DIR}/cmake/pr_common.cmake)

set(PROJ_NAME UniRender_LuxCoreRender)
pr_add_library(${PROJ_NAME} SHARED)

pr_add_dependency(${PROJ_NAME} ishared TARGET PUBLIC)
pr_add_dependency(${PROJ_NAME} util_raytracing TARGET PUBLIC)

pr_set_raw_library_path(libboost_filesystem "${DEPENDENCY_BOOST_LIBRARY_LOCATION}/libboost_filesystem-vc142-mt-x64-1_76.lib" FORCE)
pr_set_raw_library_path(libboost_iostreams "${DEPENDENCY_BOOST_LIBRARY_LOCATION}/libboost_iostreams-vc142-mt-x64-1_76.lib" FORCE)
pr_set_raw_library_path(libboost_serialization "${DEPENDENCY_BOOST_LIBRARY_LOCATION}/libboost_serialization-vc142-mt-x64-1_76.lib" FORCE)
pr_set_raw_library_path(libboost_zlib "${DEPENDENCY_BOOST_LIBRARY_LOCATION}/libboost_zlib-vc142-mt-x64-1_76.lib" FORCE)
pr_set_raw_library_path(libboost_thread "${DEPENDENCY_BOOST_LIBRARY_LOCATION}/libboost_thread-vc142-mt-x64-1_76.lib" FORCE)
pr_set_raw_library_path(libboost_regex "${DEPENDENCY_BOOST_LIBRARY_LOCATION}/libboost_regex-vc142-mt-x64-1_76.lib" FORCE)

pr_set_raw_library_path(luxcorerender_opensubdiv "${DEPENDENCY_LUXCORERENDER_LIBRARY_LOCATION}/opensubdiv.lib" FORCE)
pr_set_raw_library_path(luxcorerender_tiff "${DEPENDENCY_TIFF_LIBRARY}" FORCE)
pr_set_raw_library_path(luxcorerender_lpng "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/libpng.lib" FORCE)
pr_set_raw_library_path(luxcorerender_zlib "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/zlib1.lib" FORCE)
pr_set_raw_library_path(luxcorerender_jpeg "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/jpeg.lib" FORCE)
pr_set_raw_library_path(luxcorerender_bcd "${DEPENDENCY_LUXCORERENDER_LIBRARY_LOCATION}/bcd.lib" FORCE)
pr_set_raw_library_path(luxcorerender_blosc "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/blosc.lib" FORCE)
pr_set_raw_library_path(luxcorerender_embree3 "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/embree3.lib" FORCE)
pr_set_raw_library_path(luxcorerender_openvdb "${DEPENDENCY_LUXCORERENDER_LIBRARY_LOCATION}/openvdb.lib" FORCE)
pr_set_raw_library_path(luxcorerender_luxrays "${DEPENDENCY_LUXCORERENDER_LIBRARY_LOCATION}/luxrays.lib" FORCE)
pr_set_raw_library_path(luxcorerender_slgcore "${DEPENDENCY_LUXCORERENDER_LIBRARY_LOCATION}/slg-core.lib" FORCE)
pr_set_raw_library_path(luxcorerender_slgfilm "${DEPENDENCY_LUXCORERENDER_LIBRARY_LOCATION}/slg-film.lib" FORCE)
pr_set_raw_library_path(luxcorerender_slgkernels "${DEPENDENCY_LUXCORERENDER_LIBRARY_LOCATION}/slg-kernels.lib" FORCE)
pr_set_raw_library_path(luxcorerender_tbb "${DEPENDENCY_LUXCORERENDER_DEPS_LIBRARY_LOCATION}/tbb.lib" FORCE)

pr_add_external_dependency(${PROJ_NAME} luxcorerender LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} openimageio HEADER_ONLY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_openimageio LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_openimageio_util LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_openimagedenoise LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} openexr HEADER_ONLY PUBLIC)

pr_add_external_dependency(${PROJ_NAME} luxcorerender_tiff LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_lpng LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_zlib LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_jpeg LIBRARY PUBLIC)

# Required for color management
pr_add_external_dependency(${PROJ_NAME} util_ocio LIBRARY PUBLIC)

pr_add_external_dependency(${PROJ_NAME} luxcorerender_OpenEXR_Half LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_OpenEXR_Iex LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_OpenEXR_IlmThread LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_OpenEXR_Imath LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_OpenEXR_IlmImf LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_opencolorio LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_OpenImageIO LIBRARY PUBLIC)

pr_add_external_dependency(${PROJ_NAME} luxcorerender_libboost_thread LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_libboost_regex LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_libboost_filesystem LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_libboost_iostreams LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_libboost_zlib LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_libboost_serialization LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_libboost_wserialization LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_opensubdiv LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_bcd LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_blosc LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_embree3 LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_openvdb LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_luxrays LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_slgcore LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_slgfilm LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_slgkernels LIBRARY PUBLIC)
pr_add_external_dependency(${PROJ_NAME} luxcorerender_tbb LIBRARY PUBLIC)

target_link_libraries(${PROJ_NAME} PRIVATE
	opengl32
	glu32
	bcrypt
)

pr_add_compile_definitions(
	${PROJ_NAME} PUBLIC
		-DOIIO_STATIC_BUILD
		-DGLEW_STATIC
		-DRTUTIL_DLL
)

pr_add_headers(${PROJ_NAME} "include/")
pr_add_sources(${PROJ_NAME} "src/")

pr_finalize(${PROJ_NAME})
