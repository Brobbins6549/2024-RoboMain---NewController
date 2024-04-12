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

class Feed : public AutoStep {
    public:
    Feed(const double speed) : AutoStep("Feed") {
        m_speed = speed;
    }

    void Init() {}

    bool Execute() {
        Pow::GetInstance().Feed(m_speed);
        Intake::GetInstance().Speed1(-.05);
        return true;
    }
    private:
    double m_speed;
};