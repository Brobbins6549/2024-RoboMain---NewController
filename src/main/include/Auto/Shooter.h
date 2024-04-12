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

class Shooter : public AutoStep {
    public:
    Shooter(const double speed) : AutoStep("Shooter") {
        m_speed = speed;
    }

    void Init() {}

    bool Execute() {
        Pow::GetInstance().Shoot(-m_speed);
        return true;
    }
    private:
    double m_speed;
};