#pragma once

#include "commonauto/AutoStep.h"

//#include "Claw.h"

class SetClawDown : public AutoStep {

    public:
        SetClawDown(const double position, const double position2) : AutoStep("SetClawDown") {
            m_position = position;
            m_position2 = position2;
        }

        void Init() {}

        bool Execute() {

            //Claw::GetInstance().ClawTiltPositionDown(m_position,m_position2);
            return true;
        }

    private:
        double m_position2;
        double m_position;

};