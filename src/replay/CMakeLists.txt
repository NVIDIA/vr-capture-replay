#[[
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
]]
 

set(FOLDER_NAME replay)
set(TARGET_NAME driver_VCR_${FOLDER_NAME})
add_compile_definitions(PROJECT_NAME="${TARGET_NAME}")


add_library(${TARGET_NAME} SHARED
  driver_replay.cpp
  log.h
  OpenVR_HMDDriver.h
  OpenVR_HMDDriver.cpp
  OpenVR_ControllerDriver.h
  OpenVR_ControllerDriver.cpp
  D3D11_tex.h
  vrutil.h
  vrutil.cpp
  ../shared/logging.h
  ../shared/nvprint.hpp
  ../shared/nvprint.cpp
  ../shared/VRData.h
  ../thirdparty/cereal/cereal.hpp
  ../thirdparty/openvr/headers/openvr_driver.h
  ../thirdparty/magic_enum.hpp
  ${OPENVR_DIR}/samples/drivers/utils/driverlog/driverlog.cpp
  ${OPENVR_DIR}/samples/drivers/utils/driverlog/driverlog.h
)

add_definitions(-DDRIVER_SAMPLE_EXPORTS)

target_link_libraries(${TARGET_NAME}
  ${OPENVR_LIBRARIES}
  d3d11.lib
  dxgi.lib
)

set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 20)

# can't use TARGET_NAME here because OpenVR expects driver_replay.dll under replay directory
set_target_properties( ${TARGET_NAME} PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY_RELEASE ${OUT_DIR}/${FOLDER_NAME}/bin/win64
  RUNTIME_OUTPUT_DIRECTORY_DEBUG ${OUT_DIR}/${FOLDER_NAME}/bin/win64
)

add_custom_command(TARGET ${TARGET_NAME} POST_BUILD 
  COMMAND ${CMAKE_COMMAND} -E copy ${OPENVR_DIR}/bin/win64/openvr_api.dll ${OUT_DIR}/${FOLDER_NAME}/bin/win64/openvr_api.dll
)