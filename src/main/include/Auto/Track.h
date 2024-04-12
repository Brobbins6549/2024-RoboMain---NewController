#pragma once

#include "limelight/LimelightBack.h"
#include "swerve/src/include/SwerveTrain.h"
#include "commonauto/AutoStep.h"
#include "geo/GeoUtils.h"
#include "navX/NavX.h"

class Track : public AutoStep {
    public:
    Track() : AutoStep("Track") {}

    void Init() {
        LimelightBack::GetInstance().Normal();
    }

    bool Execute() {
        if (LimelightBack::GetInstance().getHorizontalOffset() > 2.5 || LimelightBack::GetInstance().getHorizontalOffset() < -2.5) {
            speed = 1;
        } else if (LimelightBack::GetInstance().getHorizontalOffset() > 1.7 || LimelightBack::GetInstance().getHorizontalOffset() < -1.7){
            speed = .1;
        } else {
            speed = 0;
        }
        if (LimelightBack::GetInstance().getTarget()){
            if (LimelightBack::GetInstance().getHorizontalOffset() > 1){
                SwerveTrain::GetInstance().Drive(0,0,1,true,false,speed);
            } else if (LimelightBack::GetInstance().getHorizontalOffset() < -1){
                SwerveTrain::GetInstance().Drive(0,0,-1,true,false,speed);            
            } else {
                SwerveTrain::GetInstance().Drive(0,-1,0,true,false,.7);
            }
        } else {
            SwerveTrain::GetInstance().Drive(0,-1,0,true,false,.7);
            return true;
        }
        return false;
    }
    private:
    double speed;
};