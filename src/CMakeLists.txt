#link_directories(${ZED_LIBRARY_DIR})
#link_directories(${OpenCV_LIBRARY_DIRS})
#link_directories(${CUDA_LIBRARY_DIRS})

set(UBITRACK_LIBRARY_NAME device_camera_zed)

# All cpp files for utvisualization components
FILE(GLOB UBITRACK_SRCS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
   "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

FILE(GLOB UBITRACK_HDRS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
        )

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

add_library(${UBITRACK_LIBRARY_NAME} SHARED ${UBITRACK_SRCS} ${UBITRACK_HDRS})

target_link_libraries(${UBITRACK_LIBRARY_NAME} 
    CONAN_PKG::ubitrack_core
    CONAN_PKG::ubitrack_vision
    CONAN_PKG::ubitrack_dataflow
    ${ZED_LIBS}
    )


target_include_directories(${UBITRACK_LIBRARY_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CUDA_INCLUDE_DIRS}>
  $<BUILD_INTERFACE:${ZED_INCLUDE_DIRS}>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})


# do not create symlinks to versioned names of component libraries
set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES NO_SONAME ON)

# Apply Target Properties
if(MSVC)
  if(CMAKE_CROSSCOMPILING)
    set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:secchk")
  endif()
  set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:libc /DEBUG")
  set(UBITRACK_COMPILE_FLAGS ${UBITRACK_COMPILE_FLAGS} "/bigobj ")
endif()

ubitrack_set_target_properties(${UBITRACK_LIBRARY_NAME})
ubitrack_install_component(${UBITRACK_LIBRARY_NAME})
