Getting Started {#getting_started}
============

VSM for DJI platform (vsm-dji-onboard) implemented using VSM-SDK of [Universal ground Control Software](http://www.ugcs.com/ "UgCS") and DJI Onboard SDK.

## General information {#general_information}

VSM for DJI platform is implemented using C++ language (C++14 standard) and based on the [UgCS VSM SDK](https://github.com/ugcs/vsm-cpp-sdk). 

VSM for DJI platform uses CMake as its build system to keep it platform-independent.

## How to build vsm-dji-onboard {#build_vsm}

Please follow the next instruction:

1) [Set up the build environment according to the instruction](https://github.com/ugcs/vsm-cpp-sdk#setting-up-the-build-environment-setting_up).

2) [Set up IDE if it's nessecary according to the instruction](https://github.com/ugcs/vsm-cpp-sdk#ides-setup-optional-ides_setup).

3) Checkout latest UgCS VSM SDK from the repository [https://github.com/ugcs/vsm-cpp-sdk.git](https://github.com/ugcs/vsm-cpp-sdk).

4) [Compile UgCS VSM SDK according to the instruction](https://github.com/ugcs/vsm-cpp-sdk#compiling-sdk-compiling_sdk)

5) Compile vsm-dji-onboard

		5.1 Clone the vsm-dji-onboard and common repositories:

        git clone git@github.com:UgCS/vsm-cpp-common.git
				git clone git@github.com:UgCS/dji-onboard-vsm.git
					 
		5.2 Make and enter into build directory:
		
				mkdir -p build/vsm-dji-onboard
				cd build/vsm-dji-onboard

		5.3 Create make files:

				ToDo cmake -DVSM_SDK_DIR=$HOME/install/opt/vsm-sdk -DPROTOBUF_INSTALL_DIR=~/vsm-cpp-deps/toolchain/linux/protobuf/ -DCOMMON_SOURCES=$HOME/vsm-cpp-common -G"Unix Makefiles" $HOME/vsm-dji-onboard
				
		5.4 Launch the build:

				If build suceeds you'll have vsm-ardupilot executable in current directory.

				cmake --build .


## How to run vsm-dji-onboard {#run_vsm}

### How to install vsm-dji-onboard {#install_vsm}

ToDo 

### How to configure vsm-dji-onboard {#configure_vsm}

There is a configuration file in the sources: vsm.conf.

This file contains many settings with description, but for successfully connection, we need to set up the next settings:

		vehicle.dji.app_id - ToDo
		
		vehicle.dji.app_key - ToDo
		
		vehicle.dji.serial - ToDo

