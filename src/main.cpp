// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <dji_vehicle.h>

DEFINE_DEFAULT_VSM_NAME;

int
main(int argc, char *argv[])
{
    LOG_INFO("VSM version=%d.%d.%s",
        SDK_VERSION_MAJOR,
        SDK_VERSION_MINOR,
        SDK_VERSION_BUILD);
    ugcs::vsm::Initialize(argc, argv, std::string(ugcs::vsm::Get_vsm_name()) + ".conf");
    auto vehicle = Dji_vehicle::Create();
    vehicle->Enable();
    vehicle->Wait_done();
    vehicle->Disable();
    ugcs::vsm::Terminate();
    return 0;
}
