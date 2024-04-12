#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include "Consts.h"
#include <frc/AnalogPotentiometer.h>
#include <frc/PowerDistribution.h>

class Intake {

public:
    static Intake& GetInstance() {
        static Intake* instance = new Intake(R_IntakeCANID1, R_PDH);
        return *instance;
    }

    void Speed1(double speedToSet) {
        m_Intake1->Set(speedToSet);
    }

    double Current() {
        return m_PDH->GetCurrent(2);
    }

private:
    Intake(const int R_IntakeCANID1, const int R_PDH) {
        m_Intake1 = new rev::CANSparkMax(R_IntakeCANID1, rev::CANSparkMax::MotorType::kBrushless);
        m_PDH = new frc::PowerDistribution{R_PDH, frc::PowerDistribution::ModuleType::kRev}; //talk to aiden about canid on the pdh and it having to be 1
    }

    rev::CANSparkMax* m_Intake1;
    frc::PowerDistribution *m_PDH;
};
