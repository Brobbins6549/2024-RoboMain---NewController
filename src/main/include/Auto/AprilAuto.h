#pragma once

#include "commonauto/AutoStep.h"
#include "limelight/Limelight.h"
#include "swerve/src/include/SwerveTrain.h"
#include "geo/GeoUtils.h"
#include "navX/NavX.h"
#include <math.h>
#include <chrono>

class AprilAuto : public AutoStep {

    public:
        AprilAuto() : AutoStep("AprilAuto"){

        }
        void Init() {}

        bool Execute() {
        if (Limelight::GetInstance().getTargetArea() > .14){
            ySpeeed = -.6;
        } else if (Limelight::GetInstance().getTargetArea() < .1) {
            ySpeeed = .6;
        } else {
            ySpeeed = 0;
        }
        Limelight::GetInstance().setLime(true);
        Limelight::GetInstance().April();
        frc::SmartDashboard::GetNumber("ID", Limelight::GetInstance().getID());
        if (Limelight::GetInstance().getID() == 1) {
            if (Limelight::GetInstance().getTargetArea() > .1 && Limelight::GetInstance().getTargetArea() < .14) {
            aligned = false;
            SwerveTrain::GetInstance().Drive(0,0,0,false,false,1);
            return true;
            }
            else if (Limelight::GetInstance().getHorizontalOffset() > -3 && Limelight::GetInstance().getHorizontalOffset() < 3){
            aligned = true;
            return false;
            } else {
            SwerveTrain::GetInstance().Drive(
            0,
            0,
            Limelight::GetInstance().getHorizontalOffset() / 60,
            false,
            false,
            .5
            );
            aligned = false;
            return false;
            }
        }
        if (aligned == true){
            if (Limelight::GetInstance().getHorizontalOffset() < -28 && Limelight::GetInstance().getHorizontalOffset() > -32){
            SwerveTrain::GetInstance().Drive(
                0,
                ySpeeed,
                0,
                true,
                false,
                .8
            );
            return false;
            } else {
                SwerveTrain::GetInstance().Drive(
                alignment,
                0,
                0,
                true,
                false,
                .5
                );              
                return false;
            }

            
            } else if (Limelight::GetInstance().getID() == 4){

            }
        }
    private:
    bool aligned;
    double alignment;
    double ySpeeed;
};