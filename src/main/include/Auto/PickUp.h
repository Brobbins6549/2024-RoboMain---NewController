#pragma once

#include "commonauto/AutoStep.h"
#include "limelight/Limelight.h"
#include "swerve/src/include/SwerveTrain.h"
#include "geo/GeoUtils.h"
#include "navX/NavX.h"
#include "Intake.h"
//#include "Claw.h"
#include <math.h>
#include <chrono>

class PickUp : public AutoStep {
    public:
        PickUp() : AutoStep("PickUp"){
        }
        void Init() {}
        
        bool Execute() {

            Intake::GetInstance().Speed1(.45);
            return true;
        }

    private:
};