#pragma once

#include <math.h>
#include "limelight/Limelight.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "swerve/src/include/SwerveTrain.h"
#include "Pow.h"   
class April {
public:
    static April& GetInstance() {
        static April instance;
        return instance;
    }
    void HumanPlayer(double ID) {
        Limelight::GetInstance().April();
        if (Limelight::GetInstance().getTargetArea() < .6){
            speed = 1;
        } else {
            speed = .45;
        }
        if (ID == 1) {
          frc::SmartDashboard::PutNumber("area", Limelight::GetInstance().getTargetArea());
            if (Limelight::GetInstance().getID() == 1 || Limelight::GetInstance().getID() == 2 || Limelight::GetInstance().getID() == 6) {
                if (Limelight::GetInstance().getHorizontalOffset() < -7){
                  SwerveTrain::GetInstance().Drive(1,0,0,true,false,.2);
                } else if (Limelight::GetInstance().getHorizontalOffset() > 7){
                  SwerveTrain::GetInstance().Drive(-1,0,0,true,false,.2);
                } else if (Limelight::GetInstance().getTargetArea() > 2.2){  
                  SwerveTrain::GetInstance().Drive(0,0,0,false,false,0);
                } else {
                  SwerveTrain::GetInstance().Drive(0,1,0,true,false,speed);
                }
            }
        } else if (ID == 2) {
          frc::SmartDashboard::PutNumber("area", Limelight::GetInstance().getTargetArea());
            if (Limelight::GetInstance().getID() == 9 || Limelight::GetInstance().getID() == 10 || Limelight::GetInstance().getID() == 5) {
                if (Limelight::GetInstance().getHorizontalOffset() < -7){
                  SwerveTrain::GetInstance().Drive(1,0,0,true,false,.2);
                } else if (Limelight::GetInstance().getHorizontalOffset() > 7){
                  SwerveTrain::GetInstance().Drive(-1,0,0,true,false,.2);
                } else if (Limelight::GetInstance().getTargetArea() > 13){  
                  SwerveTrain::GetInstance().Drive(0,0,0,false,false,0);
                } else {
                  SwerveTrain::GetInstance().Drive(0,1,0,true,false,speed);
                }
            }
        }

        
    }
    void Aim(){
      if (Limelight::GetInstance().getHorizontalOffset() > 3){
        if (Limelight::GetInstance().getHorizontalOffset() > 12){
          SwerveTrain::GetInstance().Drive(0,0,1,false,false,.8);
        } else if (Limelight::GetInstance().getHorizontalOffset() > 8){
          SwerveTrain::GetInstance().Drive(0,0,1,false,false,.25);
        } else if (Limelight::GetInstance().getHorizontalOffset() > 5){
          SwerveTrain::GetInstance().Drive(0,0,1,false,false,.1);
        } else {
          SwerveTrain::GetInstance().Drive(0,0,1,false,false,.05);
        }
      } else if (Limelight::GetInstance().getHorizontalOffset() < -3){
        if (Limelight::GetInstance().getHorizontalOffset() < -12){
          SwerveTrain::GetInstance().Drive(0,0,-1,false,false,.8);
        } else if (Limelight::GetInstance().getHorizontalOffset() < -8){
          SwerveTrain::GetInstance().Drive(0,0,-1,false,false,.25);
        } else if (Limelight::GetInstance().getHorizontalOffset() < -5){
          SwerveTrain::GetInstance().Drive(0,0,-1,false,false,.1);
        } else {
          SwerveTrain::GetInstance().Drive(0,0,-1,false,false,.05);
        }
      } else {
        SwerveTrain::GetInstance().Drive(0,0,0,false,false,0);
      }

    }
private:
    double speed;
};