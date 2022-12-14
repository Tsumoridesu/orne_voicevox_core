cmake_minimum_required(VERSION 3.16)

project(ONNXCore)

set(CMAKE_MACOSX_RPATH 1)

# modelファイルを読み込み
include(src/embedBin/FindEmbed.cmake)
if(EXISTS "${MODEL_DIR}/metas.json"
	AND EXISTS "${MODEL_DIR}/yukarin_s.onnx"
	AND EXISTS "${MODEL_DIR}/yukarin_sa.onnx"
	AND EXISTS "${MODEL_DIR}/decode.onnx")
	message("Models exist.")
	EMBED_TARGET(YUKARIN_S "${MODEL_DIR}/yukarin_s.onnx")
	EMBED_TARGET(YUKARIN_SA "${MODEL_DIR}/yukarin_sa.onnx")
	EMBED_TARGET(DECODE "${MODEL_DIR}/decode.onnx")
	EMBED_TARGET(METAS "${MODEL_DIR}/metas.json")
else()
	message(FATAL_ERROR "Unable to find Model. Use option -DMODEL_DIR=...")
endif()

# coreライブラリのインストール先設定。デフォルトはCMakeLists.txtと同じ位置
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "Path to install" FORCE)
endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
message("core will be installed to: ${CMAKE_INSTALL_PREFIX}")

file(GLOB_RECURSE core_sources "src/*.cpp")
# coreライブラリのビルド設定
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
add_library(core
		SHARED ${core_sources}
		${EMBED_YUKARIN_S_OUTPUTS}
		${EMBED_YUKARIN_SA_OUTPUTS}
		${EMBED_DECODE_OUTPUTS}
		${EMBED_METAS_OUTPUTS})

# -DONNXRUNTIME_DIRで指定されたパスをもとにonnxruntimeのライブラリを取得する。失敗した場合はfatal error
get_filename_component(ONNXRUNTIME_DIR ${ONNXRUNTIME_DIR} ABSOLUTE)

if(NOT DIRECTML)
	# cpu, cuda版onnxruntimeを読み込み
	if(EXISTS ${ONNXRUNTIME_DIR}/VERSION_NUMBER)
		file(READ "${ONNXRUNTIME_DIR}/VERSION_NUMBER" ONNXRUNTIME_VERSION)
		message("ONNX Runtime Version: ${ONNXRUNTIME_VERSION}")
	else()
		message(FATAL_ERROR "Unable to find ONNX Runtime. Use option -DONNXRUNTIME_DIR=...")
	endif()
	file(GLOB ONNXRUNTIME_LIBS
		"${ONNXRUNTIME_DIR}/lib/*.dylib"
		"${ONNXRUNTIME_DIR}/lib/*.dll"
		"${ONNXRUNTIME_DIR}/lib/*.lib"
		"${ONNXRUNTIME_DIR}/lib/*.so"
		"${ONNXRUNTIME_DIR}/lib/*.so.*")
	file(GLOB ONNXRUNTIME_DLLS
		"${ONNXRUNTIME_DIR}/lib/*.dll")
	set(DEPENDENT_DLLS "${DEPENDENT_DLLS};${ONNXRUNTIME_DLLS}" PARENT_SCOPE)
	target_include_directories(core
		PRIVATE ${ONNXRUNTIME_DIR}/include)
	target_link_directories(core PUBLIC ${ONNXRUNTIME_DIR}/lib)
	target_link_libraries(core PUBLIC onnxruntime)

else()
	# DirectML版onnxruntimeを読み込み
	message("-DONNXRUNTIME_DIR: ${ONNXRUNTIME_DIR}")
	if(EXISTS ${ONNXRUNTIME_DIR}/[Content_Types].xml)
		message("ONNX Runtime for DirectML - exists")
	else()
		message(FATAL_ERROR "Unable to find ONNX Runtime. Use option -DONNXRUNTIME_DIR=...")
	endif()

	file(GLOB ONNXRUNTIME_LIBS
		"${ONNXRUNTIME_DIR}/runtimes/win-${DML_ARCH}/native/*.dll"
		"${ONNXRUNTIME_DIR}/runtimes/win-${DML_ARCH}/native/*.lib")

	target_include_directories(core PRIVATE ${ONNXRUNTIME_DIR}/build/native/include)
	target_link_directories(core PUBLIC ${ONNXRUNTIME_DIR}/runtimes/win-${DML_ARCH}/native/)
	target_link_libraries(core PUBLIC onnxruntime)

	# DirectMLを読み込み
	if(EXISTS ${DIRECTML_DIR}/README.md)
		message("DirectML exists")
		file(GLOB DIRECTML_LIBS
		"${DIRECTML_DIR}/bin/${DML_ARCH}-win/*.dll"
		"${DIRECTML_DIR}/bin/${DML_ARCH}-win/*.lib")
		file(GLOB DIRECTML_DLLS
		"${DIRECTML_DIR}/bin/${DML_ARCH}-win/*.dll")
		set(DEPENDENT_DLLS "${DEPENDENT_DLLS};${DIRECTML_DLLS}" PARENT_SCOPE)

		target_include_directories(core PRIVATE ${DIRECTML_DIR}/include)
		target_link_directories(core PUBLIC ${DIRECTML_DIR}/bin/${DML_ARCH}-win/)
		target_link_libraries(core PUBLIC directml)
	else()
		message(WARNING "Unable to find DirectML. Use option -DDIRECTML_DIR=...")
	endif()

	# define DIRECTML
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D DIRECTML")
endif()

set_property(TARGET core PROPERTY CXX_STANDARD 20)
set_property(TARGET core PROPERTY POSITION_INDEPENDENT_CODE ON) # fPIC
# rpath設定
if (APPLE)
	# 参考 https://gist.github.com/robertmaynard/5750737
	set_property(TARGET core PROPERTY INSTALL_RPATH "@loader_path/")
elseif (UNIX)
	set_property(TARGET core PROPERTY INSTALL_RPATH "$ORIGIN")
endif ()

target_compile_options(core PRIVATE
	$<$<CXX_COMPILER_ID:MSVC>: /W4 /utf-8 /DVOICEVOX_CORE_EXPORTS>
	$<$<CXX_COMPILER_ID:GNU>: -Wall -Wextra  -DVOICEVOX_CORE_EXPORTS>
)

add_compile_options(TARGET core 
	$<$<CONFIG:Release>:$<CXX_COMPILER_ID:MSVC>: /O2>
	$<$<CONFIG:Release>:$<CXX_COMPILER_ID:GNU>: -O2>
)

target_include_directories(core
	PRIVATE ../open_jtalk/src/jpcommon
	PRIVATE ../open_jtalk/src/mecab/src
	PRIVATE ../open_jtalk/src/mecab2njd
	PRIVATE ../open_jtalk/src/mecab-naist-jdic
	PRIVATE ../open_jtalk/src/njd
	PRIVATE ../open_jtalk/src/njd2jpcommon
	PRIVATE ../open_jtalk/src/njd_set_accent_phrase
	PRIVATE ../open_jtalk/src/njd_set_accent_type
	PRIVATE ../open_jtalk/src/njd_set_digit
	PRIVATE ../open_jtalk/src/njd_set_long_vowel
	PRIVATE ../open_jtalk/src/njd_set_pronunciation
	PRIVATE ../open_jtalk/src/njd_set_unvoiced_vowel
	PRIVATE ../open_jtalk/src/text2mecab)
target_link_libraries(core PUBLIC openjtalk)


# GCC 9.0以前ではstd::filesystemを使うためにリンクが必要 (https://gitlab.kitware.com/cmake/cmake/-/issues/17834)
target_link_libraries(core PRIVATE $<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.0>>:stdc++fs>)

# cmake --installを行うとcoreライブラリ、onnxruntimeライブラリ、core.hがインストール先のlibフォルダにコピーされる
install(TARGETS core
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib)
install(FILES ${ONNXRUNTIME_LIBS} ${CMAKE_CURRENT_SOURCE_DIR}/src/core.h
	DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

if(DIRECTML AND EXISTS ${DIRECTML_DIR}/README.md)
	install(FILES ${DIRECTML_LIBS} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
endif()
