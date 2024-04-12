#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include "Consts.h"
#include <frc/Solenoid.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/PowerDistribution.h>
#include "limelight/Limelight.h"

class Climber{
    public:
        static Climber& GetInstance() {
            static Climber* instance = new Climber(R_ClimberCANID1,R_ClimberCANID2);
            return *instance;
        }
        void speed(double speed) {
            m_Climber1->Set(speed);
            m_Climber2->Set(speed);
        }
    private:
    Climber(const int R_ClimberCANID1, const int R_ClimberCANID2) {
        m_Climber1 = new rev::CANSparkMax(R_ClimberCANID1, rev::CANSparkMax::MotorType::kBrushless);
        m_Climber2 = new rev::CANSparkMax(R_ClimberCANID2, rev::CANSparkMax::MotorType::kBrushless);
    }
    rev::CANSparkMax* m_Climber1;
    rev::CANSparkMax* m_Climber2;
};