Getting Started {#getting_started}
============

VSM for DJI platform (vsm-dji-onboard) implemented using VSM-SDK of [Universal ground Control Software](http://www.ugcs.com/ "UgCS") and DJI Onboard SDK.

## General information {#general_information}

VSM for DJI platform is implemented using C++ language (C++14 standard) and based on the [UgCS VSM SDK](https://github.com/ugcs/vsm-cpp-sdk). 

VSM for DJI platform uses CMake as its build system to keep it platform-independent.

## Supported drones {#build_vsm}

The VSM for DJI platform supports the next drones: A3、N3、M100、M210 V1、M600、M600 Pro, M210 V2.

The next DJI OSDK version 4.x is not supported right now, but it can be implemented. The DJI M300 RTK support will be implemented after changing OSDK to 4.x version.

## How to build vsm-dji onboard on Docker container

Please follow the next instruction:

1) Install Docker application.

2) Checkout the current repository.

3) Checkout the branch "dev-dji-onboard-vsm" of UgCS VSM SDK from the repository [https://github.com/ugcs/vsm-cpp-sdk.git](https://github.com/ugcs/vsm-cpp-sdk).

4) Checkout the 3.9 tag for the DJI Onboard SDK from the repository [https://github.com/dji-sdk/Onboard-SDK.git](https://github.com/dji-sdk/Onboard-SDK.git).
	
5) Build the docker container:
	
		cd dji-onboard-vsm/docker-cross-armhf
		docker build . -t docker-cross-armhf
	
6) Build the dji-vsm-onboard:

 	6.1 Run the next command: 
 
		docker run -it -u root -v $work_dir/git/vsm-cpp-sdk:/vsm-cpp-sdk -v $work_dir/Onboard-SDK:/dji-onboard-sdk -v $work_dir/dji-onboard-vsm:/dji-onboard-vsm -v $work_dir/dji-onboard-vsm/out:/out docker-cross-armhf ./build.sh $architecture
	
		Where:
		$work_dir/git/vsm-cpp-sdk - path to the local copy of the https://github.com/ugcs/vsm-cpp-sdk
		$work_dir/Onboard-SDK - path to the local copy of the https://github.com/dji-sdk/Onboard-SDK.git
		$work_dir/dji-onboard-vsm - path to the local copy of the https://github.com/ugcs/dji-onboard-vsm.git
		$work_dir/dji-onboard-vsm/out - path to the output folder
		$architecture - select one of available architecture x86, arm64 or armhf
		
	6.2 After successful build you will have two files in the output folder: vsm-dji-onboard and vsm-dji-onboard.conf.

## How to build vsm-dji-onboard manually {#build_vsm}

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

		cmake -DVSM_SDK_DIR=$HOME/install/opt/vsm-sdk -DPROTOBUF_INSTALL_DIR=~/vsm-cpp-deps/toolchain/linux/protobuf/ -DCOMMON_SOURCES=$HOME/vsm-cpp-common -G"Unix Makefiles" $HOME/vsm-dji-onboard
				
	5.4 Launch the build:

		If build suceeds you'll have vsm-ardupilot executable in current directory.

		cmake --build .


## How to run vsm-dji-onboard {#run_vsm}

### How to configure vsm-dji-onboard {#configure_vsm}

There is a configuration file in the sources: vsm.conf.

This file contains many settings with description, but for successfully connection, we need to set up the next settings:

	vehicle.dji.app_id - DJI OSDK App IP, please find more info there https://developer.dji.com/onboard-sdk/
		
	vehicle.dji.app_key - DJI OSDK App Key, please find more info there https://developer.dji.com/onboard-sdk/
		
	vehicle.dji.serial - serial device port connected with DJI Autopilot. (ex. /dev/ttyUSB0 )

### How to run vsm-dji-onboard {#run_vsm}
		
Please execute the next command: 
		
	vsm-dji-onboard --config vsm-dji-onboard.conf

After successful build and configuration, you could connect this vsm to UgCS server. Please run the server and vsm in the same network and your vehicle will be availiable in UgCS.
