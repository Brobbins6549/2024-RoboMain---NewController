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
#include "Intake.h"

class Feed3 : public AutoStep {
    public:
    Feed3(const double speed) : AutoStep("Feed3") {
        m_speed = speed;
    }

    void Init() {}

    bool Execute() {
        Pow::GetInstance().Feed(m_speed);
        return true;
    }
    private:
    double m_speed;
};