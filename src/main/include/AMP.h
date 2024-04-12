#pragma once

#include <math.h>
#include "Intake.h"
//#include "Claw.h"
#include "limelight/Limelight.h"
#include <frc/smartdashboard/SmartDashboard.h>

class AMP {
public:
    static AMP& GetInstance() {
        static AMP instance;
        return instance;
    }


private:

};
