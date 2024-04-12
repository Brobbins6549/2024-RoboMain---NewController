#pragma once

#include "commonauto/AutoStep.h"
#include "limelight/Limelight.h"
#include "swerve/src/include/SwerveTrain.h"
#include "geo/GeoUtils.h"
#include "navX/NavX.h"
#include "Pow.h"
//#include "Claw.h"
#include <math.h>
#include <chrono>

class StopShooter : public AutoStep {
    public:
    StopShooter() : AutoStep("StopShooter") {}

    void Init() {}

    bool Execute() {
        Pow::GetInstance().Stop();
        return true;
    }
    private:
};