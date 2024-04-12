#pragma once

#include "commonauto/AutoStep.h"
#include "limelight/Limelight.h"
#include "swerve/src/include/SwerveTrain.h"
#include "geo/GeoUtils.h"
#include "navX/NavX.h"
#include <math.h>
#include <chrono>

class limelightSonar : public AutoStep {
    public:
        limelightSonar() : AutoStep("LimelightSonar") {
        }
        void Init() {}

    //bool Execute() {}
    
    private:

};