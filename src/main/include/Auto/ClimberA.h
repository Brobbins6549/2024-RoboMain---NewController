#pragma once

#include "commonauto/AutoStep.h"

#include "Climber.h"

class Climb : public AutoStep {

    public:
        Climb(const double speed) : AutoStep("Climb") {

            m_speed = speed;
        }

        void Init() {}

        bool Execute() {

            Climber::GetInstance().speed(m_speed);
            return true;
        }

    private:
        double m_speed;
};