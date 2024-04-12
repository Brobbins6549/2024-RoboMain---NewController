#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include "Consts.h"
#include <frc/Solenoid.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/PowerDistribution.h>
#include "limelight/LimelightBack.h"
#include "Pow.h"
#include "swerve/src/include/SwerveTrain.h"

class Item {
    public:
    static Item& GetInstance(){
        static Item* instance = new Item();
        return *instance;
    }
    double track(){
        if (LimelightBack::GetInstance().getTarget()){
            if (LimelightBack::GetInstance().getHorizontalOffset() > 3){
                return .2;
            } else if (LimelightBack::GetInstance().getHorizontalOffset() < -3){
                return -.2;
            } else {
                return 0;
            }
        } else {
            return 0;
        }
    }
    private:
};