#pragma once

#include "commonauto/AutoStep.h"
#include "Tests.h"
//#include "Claw.h"

class SetClaw : public AutoStep {

    public:
        SetClaw(const double speed) : AutoStep("SetClaw") {

            m_speed = speed;
        }

        void Init() {}

        bool Execute() {

            Tests::GetInstance().ClawTilt(m_speed);
            return true;
        }

    private:
        double m_speed;
};