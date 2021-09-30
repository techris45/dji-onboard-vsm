#!/bin/sh
export VER_MAJOR=4
export VER_MINOR=4
export VER_BUILD=1

export BUILD_DIR=/build
export CPP_SDK_DIR=/vsm-cpp-sdk
export DJI_SDK_SRC_DIR=/dji-onboard-sdk
export VSM_SRC_DIR=/dji-onboard-vsm
export DJI_SDK_OUT_DIR=$BUILD_DIR/dji-sdk-build
export SDK_OUT_DIR=$BUILD_DIR/cpp-sdk-build
export VSM_OUT_DIR=$BUILD_DIR/vsm-build
export RESULT_DEB_NAME="ugcs-vsm-djionboard-linux-$VER_MAJOR.$VER_MINOR.$VER_BUILD.deb"
export ARCHITECTURE=$1

if [ -z "$1" ] || [ "$1" = "x86" ]
then
    echo 'Compile x86 architecture'
elif [ $ARCHITECTURE = 'arm64' ]
then
    echo 'Compile for ARM64 architecture'
    export TOOLCHAIN_FILE=$PI_HOME/toolchain-arm64.cmake
    export RESULT_DEB_NAME="ugcs-vsm-djionboard-linux-arm64-$VER_MAJOR.$VER_MINOR.$VER_BUILD.deb"
    #export FLAGS=-DCMAKE_CXX_FLAGS=-I/pitools/arm-unknown-linux-gnueabihf/arm-unknown-linux-gnueabihf/sysroot/usr/include/
elif [ $ARCHITECTURE = 'armhf' ]
then
    echo 'Compile for ARMHF architecture'
    export TOOLCHAIN_FILE=$PI_HOME/pi.cmake
    export RESULT_DEB_NAME="ugcs-vsm-djionboard-linux-armhf-$VER_MAJOR.$VER_MINOR.$VER_BUILD.deb"
    export FLAGS=-DCMAKE_CXX_FLAGS=-I/pitools/arm-unknown-linux-gnueabihf/arm-unknown-linux-gnueabihf/sysroot/usr/include/
else
    echo "Unknown architecure $1. (arm64, armhf and x86 are supported)"
    exit
fi

echo 'Compile DJI ONBOARD SDK'
#rm -fr $DJI_SDK_OUT_DIR || true
mkdir -p $DJI_SDK_OUT_DIR && cd $DJI_SDK_OUT_DIR
cmake $DJI_SDK_SRC_DIR -DCMAKE_TOOLCHAIN_FILE=$TOOLCHAIN_FILE
make install

echo 'Compile UgCS CPP SDK'
#rm -rf $SDK_OUT_DIR || true
mkdir -p $SDK_OUT_DIR && cd $SDK_OUT_DIR
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_TOOLCHAIN_FILE=$TOOLCHAIN_FILE -DPROTOBUF_INSTALL_DIR=/usr/local/lib/ $FLAGS -DCROSS_COMPILE=true -DPROTOBUF_SOURCE_ROOT=/build/protobuf $CPP_SDK_DIR
make install

echo 'Compile UgCS onboard vsm for DJI'
#rm -fr $VSM_OUT_DIR || true
mkdir -p $VSM_OUT_DIR && cd $VSM_OUT_DIR
cmake -DCMAKE_BUILD_TYPE=RELEASE -DUGCS_PACKAGE_VERSION_MAJOR=$VER_MAJOR -DUGCS_PACKAGE_VERSION_MINOR=$VER_MINOR -DUGCS_PACKAGE_VERSION_BUILD=$VER_BUILD -DCMAKE_TOOLCHAIN_FILE=$TOOLCHAIN_FILE -DCROSS_COMPILE=true -DVSM_SDK_DIR=/usr/local/opt/vsm-sdk $FLAGS -DCPACK_DEBIAN_PACKAGE_ARCHITECTURE=$ARCH $VSM_SRC_DIR
make 

cd $VSM_OUT_DIR && mv vsm-dji-onboard* /out/$ARCH