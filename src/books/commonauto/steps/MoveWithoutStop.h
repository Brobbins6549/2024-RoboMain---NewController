#pragma once

#include "commonauto/AutoStep.h"
#include "StepConsts.h"
#include "swerve/src/include/SwerveTrain.h"

class MoveWithoutStop : public AutoStep {

    public:
        MoveWithoutStop(const double x, const double y, const double timeToDrive, const bool hold, const bool jerk, const double speed) 
        : AutoStep("MoveWithoutStop"), m_x(x), m_y(y), m_timeToDrive(timeToDrive), m_hold(hold), m_jerk(jerk), m_speed(speed) {}
        void Init() {
            m_startTime = frc::GetTime().value();
            std::cout << NavX::GetInstance().getYawFull() << std::endl;
        }
        bool Execute() {
            double delta = frc::GetTime().value() - m_startTime;
            if (delta <= m_timeToDrive){
                SwerveTrain::GetInstance().Drive(m_x,m_y,0,m_hold,m_jerk,m_speed);
            } else {
                return true;
            }
            return false;
        }
    private:
    double m_x;
    double m_y;
    double m_timeToDrive;
    bool m_hold;
    bool m_jerk;
    double m_startTime;
    double m_speed;
};