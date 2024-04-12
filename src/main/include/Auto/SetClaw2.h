#pragma once

#include "commonauto/AutoStep.h"

//#include "Claw.h"

class SetClawUp : public AutoStep {

    public:
        SetClawUp(const double position, const double position2) : AutoStep("SetClawUp") {

            m_position = position;
            m_position2 = position2;
        }

        void Init() {}

        bool Execute() {

            //Claw::GetInstance().ClawTiltPositionUp(m_position,m_position2);
            return true;
        }

    private:
        double m_position;
        double m_position2;
};