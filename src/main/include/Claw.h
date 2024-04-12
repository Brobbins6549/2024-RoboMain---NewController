/*#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include "Consts.h"
#include <frc/Solenoid.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/PowerDistribution.h>
#include "limelight/Limelight.h"
#include "Pow.h"
#include "swerve/src/include/SwerveTrain.h"

enum ClawState {
    IDLE,
    MOVING_UP,
    MOVING_DOWN
};

class Claw {
public:
    static Claw& GetInstance() {
        static Claw* instance = new Claw(R_ClawActuatorCANID1,R_ClawActuatorCANID2, R_ClawPotentiometer1, R_ClawPotentiometer2);
        return *instance;
    }

    void ClawTiltSpeed1(double speedToSet) {
        m_ClawActuator1->Set(speedToSet);
    }
    void ClawTiltSpeed2(double speedToSet) {
        m_ClawActuator2->Set(speedToSet);      
    }
    void ClawTiltPositionUp(double distanceToSet, double yes) {
        m_TargetPotentiometerValue = distanceToSet;
        m_TargetPotentiometerValue2 = yes;
        m_CurrentState = MOVING_UP;
    }

    void ClawTiltPositionDown(double distanceToSet, double yes) {
        m_TargetPotentiometerValue = distanceToSet;
        m_TargetPotentiometerValue2 = yes;
        m_CurrentState = MOVING_DOWN;
    }

    void Idle() {
        m_CurrentState = IDLE;
    }

    void TrackBlue() {
        if (Limelight::GetInstance().getID() == 7 || Limelight::GetInstance().getID() == 8 || Limelight::GetInstance().getID() == 6){
            if (m_ClawPotentiometer1->Get() < 0.11 || m_ClawPotentiometer2->Get() > .87) {
                Claw::GetInstance().ClawTiltPositionDown(.1,.86);
            } else if (m_ClawPotentiometer1->Get() > 0.09 || m_ClawPotentiometer2->Get() < 0.85) {
                Claw::GetInstance().ClawTiltPositionUp(.06,.86);
            } else {
                Claw::GetInstance().Idle();
            } 
        } else {
            if (m_ClawPotentiometer1->Get() < 0.155 || m_ClawPotentiometer2->Get() > 0.81) {
                Claw::GetInstance().ClawTiltPositionDown(.145,.8);
            } else if (m_ClawPotentiometer1->Get() > 0.135 || m_ClawPotentiometer2->Get() < 0.79) {
                Claw::GetInstance().ClawTiltPositionUp(.145,.8);
            } else {
                Claw::GetInstance().Idle();
            }             
        }
        Claw::GetInstance().Update();
    }
    void TrackRed() {
        if (Limelight::GetInstance().getID() == 4 || Limelight::GetInstance().getID() == 3 || Limelight::GetInstance().getID() == 5){
            if (m_ClawPotentiometer1->Get() > 0.11 || m_ClawPotentiometer2->Get() > .87) {
                Claw::GetInstance().ClawTiltPositionDown(.1,.86);
            } else if (m_ClawPotentiometer1->Get() < 0.09 || m_ClawPotentiometer2->Get() < 0.85) {
                Claw::GetInstance().ClawTiltPositionUp(.06,.86);
            } else {
                Claw::GetInstance().Idle();
            } 
        } else {
            if (m_ClawPotentiometer1->Get() < 0.135 || m_ClawPotentiometer2->Get() > 0.81) {
                Claw::GetInstance().ClawTiltPositionDown(.145,.8);
            } else if (m_ClawPotentiometer1->Get() > 0.155 || m_ClawPotentiometer2->Get() < 0.79) {
                Claw::GetInstance().ClawTiltPositionUp(.145,.8);
            } else {
                Claw::GetInstance().Idle();
            }             
        }

        Claw::GetInstance().Update();    
    }
    void AMP() {
        if (m_ClawPotentiometer1->Get() > .21){
            Claw::GetInstance().ClawTiltPositionDown(.2,.78);
        } else if (m_ClawPotentiometer1->Get() < .19) {
            Claw::GetInstance().ClawTiltPositionUp(.2,.78);
        } else {
            Claw::GetInstance().Idle();
        }
        Claw::GetInstance().Update();
    }
    void Update() {
        
        frc::SmartDashboard::PutNumber("Top Actuator", m_ClawPotentiometer1->Get());
        frc::SmartDashboard::PutNumber("Bottom Actuator", m_ClawPotentiometer2->Get());
        switch (m_CurrentState) {
            case IDLE:
                frc::SmartDashboard::PutString("Update", "Idle");
                m_ClawActuator1->Set(0);
                m_ClawActuator2->Set(0);
                break;

            case MOVING_UP:
                frc::SmartDashboard::PutString("Update", "Up");
                if (m_ClawPotentiometer1->Get() > m_TargetPotentiometerValue && m_ClawPotentiometer2->Get() < m_TargetPotentiometerValue2) {
                    Claw::GetInstance().Idle();
                    Claw::GetInstance().Update();
                } else if (m_ClawPotentiometer1->Get() > m_TargetPotentiometerValue) {
                    m_ClawActuator2->Set(1);
                    m_ClawActuator1->Set(0);
                } else if(m_ClawPotentiometer2->Get() < m_TargetPotentiometerValue2) {
                    m_ClawActuator1->Set(1);
                    m_ClawActuator2->Set(0);
                } else {
                    m_ClawActuator1->Set(1);
                    m_ClawActuator2->Set(1);
                }
                break;

            case MOVING_DOWN:
                frc::SmartDashboard::PutString("Update", "Down");
                if (m_ClawPotentiometer1->Get() < m_TargetPotentiometerValue && m_ClawPotentiometer2->Get() > m_TargetPotentiometerValue2) {
                    Claw::GetInstance().Idle();
                    Claw::GetInstance().Update();
                } else if (m_ClawPotentiometer1->Get() < m_TargetPotentiometerValue) {
                    m_ClawActuator2->Set(-1);
                    m_ClawActuator1->Set(0);
                } else if(m_ClawPotentiometer2->Get() > m_TargetPotentiometerValue2) {
                    m_ClawActuator1->Set(-1);
                    m_ClawActuator2->Set(0);
                } else {
                    m_ClawActuator1->Set(-1);
                    m_ClawActuator2->Set(-1);
                }
                break;
        }
    }

private:
    Claw(const int R_ClawActuatorCANID1, const int R_ClawActuatorCANID2, const int R_ClawPotentiometer1, const int R_ClawPotentiometer2) {
        m_ClawActuator1 = new rev::CANSparkMax(R_ClawActuatorCANID1, rev::CANSparkMax::MotorType::kBrushed);
        m_ClawActuator2 = new rev::CANSparkMax(R_ClawActuatorCANID2, rev::CANSparkMax::MotorType::kBrushed);
        m_ClawPotentiometer1 = new frc::AnalogPotentiometer(1, 1.0, 0.0);
        m_ClawPotentiometer2 = new frc::AnalogPotentiometer(2, 1.0, 0.0);
        
    }

    rev::CANSparkMax* m_ClawActuator1;
    rev::CANSparkMax* m_ClawActuator2;
    frc::AnalogPotentiometer* m_ClawPotentiometer1;
    frc::AnalogPotentiometer* m_ClawPotentiometer2;
    double m_TargetPotentiometerValue;
    double m_TargetPotentiometerValue2;
    double Value;
    double ValueBottom;
    double speed;
    double turn;
    ClawState m_CurrentState;
};*/
