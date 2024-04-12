#pragma once

#include "commonauto/AutoStep.h"
#include "StepConsts.h"
#include "navX/NavX.h"
#include "swerve/src/include/SwerveTrain.h"
#include "geo/GeoUtils.h"

class MoveAndTurn : public AutoStep {

    public:
        MoveAndTurn(const double x, const double y, const double angle, const double timeToDrive, const bool hold, const bool jerk) 
            : AutoStep("MoveAndTurn"), m_x(x), m_y(y), m_targetAngle(angle), m_totalTimeToDrive(timeToDrive), m_hold(hold), m_jerk(jerk) {}

        void Init() {
            m_startTime = frc::GetTime().value();
            std::cout << NavX::GetInstance().getYawFull() << std::endl;
        }

        bool Execute() {
            const double kP = 0.1;
            double delta = frc::GetTime().value() - m_startTime;
            double angleDelta = GeoUtils::MinDistFromDelta(m_targetAngle - NavX::GetInstance().getYawFull(), 360);
            double turnSpeed = kP * angleDelta;
            double speed = 0;


            // calculate movement speed based on time and jerk
            if (!m_jerk) {
                
                if (delta <= m_totalTimeToDrive / 2.0) {

                    speed = (pow(5, delta) - 1) / (double)(5 - 1);
                }
                else if (delta > m_totalTimeToDrive / 2.0) {

                    speed = (pow(5, m_totalTimeToDrive - delta) - 1) / (double)(5 - 1);
                }
                if (speed > 1) {

                    speed = 1;
                }
            }
            else {

                if (delta > m_totalTimeToDrive / 2.0) {

                    speed = (pow(5, m_totalTimeToDrive - delta) - 1) / (double)(5 - 1);
                }
                else {
                    
                    speed = 1;
                }
            }

            // calculate turn speed based on angle difference
            if (abs(angleDelta) > TURN_TO_ANGLE_ABSOLUTE__TOLERANCE) {

                turnSpeed = (pow(5, 1 / 90.0 * abs(angleDelta)) - 1) / (5 - 1);
                if (turnSpeed < 0.25) {

                    turnSpeed = 0.1;
                }
                if (angleDelta < 0) {

                    turnSpeed *= -1;
                }
                if (turnSpeed > 1) {

                    turnSpeed = 1;
                }
                else if (turnSpeed < -1) {

                    turnSpeed = -1;
                }
            }
            else {
                turnSpeed = 0;
            }

            // drive the robot
            SwerveTrain::GetInstance().Drive(m_x, m_y, turnSpeed, m_hold, (turnSpeed == 0), speed * AUTO_EXECUTION_CAP);
            frc::SmartDashboard::PutNumber("Speed", turnSpeed);
            // check if the time has elapsed and return accordingly
            if (delta >= m_totalTimeToDrive) {

                SwerveTrain::GetInstance().Stop();
                return true;
            }

            return false;
        }

    private:
        double m_x;
        double m_y;
        double m_targetAngle;
        double m_totalTimeToDrive;
        bool m_hold;
        bool m_jerk;
        double m_startTime;
};
