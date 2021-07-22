cmake_minimum_required (VERSION 2.6)

project (vsm-dji-onboard)

# variables to ease creation of OEM builds.
# UGCS_OEM_NAME      : OEM name used in user visible strings, mostly GUI. This must be a single word string.
# UGCS_OEM_ID        : Used in internal components in directory or package names.

if (NOT UGCS_OEM_NAME)
    set(UGCS_OEM_NAME "$ENV{UGCS_OEM_NAME}")
    if (NOT UGCS_OEM_NAME)
        set(UGCS_OEM_NAME "UgCS")
    endif ()
endif ()

if (NOT "${UGCS_OEM_NAME}" MATCHES "[a-z]|[A-Z]|[0-9]*")
    message (FATAL_ERROR "UGCS_OEM_NAME (${UGCS_OEM_NAME}) must contain only aphanumeric chars.")
endif ()

if (NOT UGCS_OEM_ID)
    set(UGCS_OEM_ID "$ENV{UGCS_OEM_ID}")
    if (NOT UGCS_OEM_ID)
        set(UGCS_OEM_ID "${UGCS_OEM_NAME}")
    endif()
endif ()

# Force lowercase OEM ID.
string(TOLOWER ${UGCS_OEM_ID} UGCS_OEM_ID)

message ("UGCS_OEM_NAME      = ${UGCS_OEM_NAME}")
message ("UGCS_OEM_ID        = ${UGCS_OEM_ID}")

# Deal with path to SDK
if (NOT DEFINED VSM_SDK_DIR)
    if (DEFINED ENV{VSM_SDK_DIR})
        set(VSM_SDK_DIR $ENV{VSM_SDK_DIR})
    else()
        # Default SDK path
        if(WIN32)
            set(VSM_SDK_DIR "C:/UGCS/vsm-sdk")
        elseif(APPLE)
            set(VSM_SDK_DIR "/Applications/${UGCS_OEM_NAME}/vsm-sdk")
        elseif(UNIX)
            set(VSM_SDK_DIR "/opt/${UGCS_OEM_ID}/vsm-sdk")
        endif()
    endif()
endif()

# We are not going to install locally, so 
# make sure package always contains the correct paths.
set (UGCS_USE_PACKAGING_INSTALL_PREFIX yes)

# Tell the build scripts to install vsm as systemd service
set(UGCS_INSTALL_AS_SERVICE yes)

set(UGCS_PACKAGE_SUMMARY "UgCS vehicle specific module (VSM) for DJI using Onboard SDK")
set(UGCS_VSM_USER_GUIDE_LATEX_TITLE "DJI VSM User Guide")
set(CMAKE_MODULE_PATH "${VSM_SDK_DIR}/share/cmake")
include("ugcs/vsm")

Build_vsm_config(${CMAKE_SOURCE_DIR}/vsm.conf)

# Bring in DJI SDK
if(NOT DJI_ONBOARD_SDK_DIR)
    set(DJI_ONBOARD_SDK_DIR "/opt/ugcs/dji-sdk")
endif()
set(CMAKE_PREFIX_PATH ${DJI_ONBOARD_SDK_DIR} ${CMAKE_PREFIX_PATH})
find_package(DJIOSDK REQUIRED)
include_directories ("include" ${DJIOSDK_INCLUDE_DIRS})
set (VSM_LIBS ${VSM_LIBS} ${DJIOSDK_LIBRARIES})

file(GLOB SOURCES "src/*.cpp" "src/include/*.h")

Build_vsm()
