# Include python

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
set(PYTHON_LIBRARIES ${Python3_LIBRARIES})
set(PYTHON_INCLUDE_DIRS ${Python3_INCLUDE_DIRS})
set(PYTHON_LIBRARY_DIRS ${Python3_LIBRARY_DIRS})
set(PYTHON_VERSION_STRING ${Python3_VERSION})
set(PYTHON_VERSION_MAJOR ${Python3_VERSION_MAJOR})
set(PYTHON_VERSION_MINOR ${Python3_VERSION_MINOR})
set(PYTHON_VERSION_PATCH ${Python3_VERSION_PATCH})
set(PYTHONINTERP_FOUND ${Python3_Interpreter_FOUND})

message(STATUS "python version ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
message(STATUS "python libraries ${PYTHON_LIBRARIES}")

include_directories(${PYTHON_INCLUDE_DIRS})

if (WIN32)
	set(PYLIB_SUFFIX .pyd)
else()
	set(PYLIB_SUFFIX .so)
endif()

add_definitions(-DSLVS_STATIC_LIB)

set_source_files_properties(../slvs.i PROPERTIES CPLUSPLUS ON)
include_directories(../ ../../../include/)

# Add swig module
if (${CMAKE_VERSION} VERSION_LESS "3.8.0")
    swig_add_module(slvs python ../slvs.i)
else()
    swig_add_library(slvs LANGUAGE python SOURCES ../slvs.i)
endif()

if (APPLE)
    swig_link_libraries(slvs "-Wl,-undefined,dynamic_lookup" slvs_static_excp)
elseif(WIN32)
    swig_link_libraries(slvs ${PYTHON_LIBRARIES} slvs_static_excp)
else ()
    swig_link_libraries(slvs slvs_static_excp)
endif ()

if (NOT ENABLE_GUI)
    message(STATUS "python package")
    install(TARGETS _slvs LIBRARY DESTINATION py_slvs)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/slvs.py DESTINATION py_slvs)
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py DESTINATION py_slvs)
endif( NOT ENABLE_GUI)
