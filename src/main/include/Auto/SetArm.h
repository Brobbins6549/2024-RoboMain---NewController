#pragma once

#include "commonauto/AutoStep.h"

#include "Intake.h"

class SetArm : public AutoStep {

    public:
        SetArm(const double speed) : AutoStep("SetArm") {

            m_speed = speed;
        }

        void Init() {}

        bool Execute() {

            Intake::GetInstance().Speed1(m_speed);
            return true;
        }

    private:
        double m_speed;
};