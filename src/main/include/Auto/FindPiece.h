#pragma once

#include "commonauto/AutoStep.h"
#include "limelight/Limelight.h"
#include "swerve/src/include/SwerveTrain.h"
#include "geo/GeoUtils.h"
#include "navX/NavX.h"
#include "Intake.h"
//#include "Claw.h"
#include <math.h>
#include <chrono>

class FindPiece : public AutoStep {
    public:
        FindPiece(const double turn) : AutoStep("FindPiece"){
            m_turn = turn;
        }
        void Init() {
            Limelight::GetInstance().Items();
        }

        bool Execute() {
            frc::SmartDashboard::PutString("Item", Limelight::GetInstance().object());
            if (Limelight::GetInstance().getTargetArea() > 0.0001) {
                if (Limelight::GetInstance().getHorizontalOffset() > 3) {
                    SwerveTrain::GetInstance().Drive(0,0,1,true,false,.05);
                } else if (Limelight::GetInstance().getHorizontalOffset() < -3){
                    SwerveTrain::GetInstance().Drive(0,0,-1,true,false,.05);
                } else {
                    SwerveTrain::GetInstance().Drive(0,0,0,true,false,0);
                    return true;
                }
            } else {
                SwerveTrain::GetInstance().Drive(0,0,m_turn,false,false,.12);
            }
            return false;
        }
    private:
        double m_turn;
};