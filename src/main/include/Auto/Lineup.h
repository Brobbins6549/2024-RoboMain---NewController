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

class Lineup : public AutoStep {
    public:
        Lineup() : AutoStep("Lineup"){}
        void Init() {
            Limelight::GetInstance().April();
        }

        bool Execute() {

            if (Limelight::GetInstance().getID() == 7 || Limelight::GetInstance().getID() == 4){
                if (Limelight::GetInstance().getHorizontalOffset() > 2){
                    if (Limelight::GetInstance().getVerticalOffset() < -3 && Limelight::GetInstance().getVerticalOffset() > -6){
                        SwerveTrain::GetInstance().Drive(0,0,.3,true,false,1);
                    } else if (Limelight::GetInstance().getVerticalOffset() > -3){
                        SwerveTrain::GetInstance().Drive(0,1,.3,true,false,1);
                    } else if (Limelight::GetInstance().getVerticalOffset() < -6){
                        SwerveTrain::GetInstance().Drive(0,-1,.3,true,false,1);
                    }
                } else if (Limelight::GetInstance().getHorizontalOffset() < -2){
                    if (Limelight::GetInstance().getVerticalOffset() > 3 && Limelight::GetInstance().getVerticalOffset() < 6){
                        SwerveTrain::GetInstance().Drive(0,0,-.3,true,false,1);
                    } else if (Limelight::GetInstance().getVerticalOffset() < 3){
                        SwerveTrain::GetInstance().Drive(0,1,-.3,true,false,1);
                    } else if (Limelight::GetInstance().getVerticalOffset() > 6){
                        SwerveTrain::GetInstance().Drive(0,-1,-.3,true,false,1);
                    }
                } else if (Limelight::GetInstance().getVerticalOffset() < -3 && Limelight::GetInstance().getVerticalOffset() > -6){
                    SwerveTrain::GetInstance().Drive(0,0,0,true,false,0);
                    return true;
                } else if (Limelight::GetInstance().getVerticalOffset() > -3){
                    SwerveTrain::GetInstance().Drive(0, 1,-.3,true,false,1);
                } else if (Limelight::GetInstance().getVerticalOffset() < -6){
                    SwerveTrain::GetInstance().Drive(0,-1,-.3,true,false,1);
                }
            }
            return false;
            
            
        }
    private:
};