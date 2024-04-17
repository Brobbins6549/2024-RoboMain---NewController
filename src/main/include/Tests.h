#pragma once
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

class Tests {
    public:
    static Tests& GetInstance() {
        static Tests* instance = new Tests(R_ClawActuatorCANID1, R_ClawActuatorCANID2);
        return *instance;
    }
    void ClawTilt(double speed){
        m_ClawActuator1->Set(speed);
        m_ClawActuator2->Set(speed);
        frc::SmartDashboard::PutNumber("Top", m_ClawPotentiometer1->Get());
        frc::SmartDashboard::PutNumber("Bottom", m_ClawPotentiometer2->Get());
    }
    void TrackRed(){
        frc::SmartDashboard::PutNumber("Top", m_ClawPotentiometer1->Get());
        frc::SmartDashboard::PutNumber("Bottom", m_ClawPotentiometer2->Get());
        if (0.0067092916 * Limelight::GetInstance().getVerticalOffset() + .9437390001 > .92){
            value = .92;
        } else if (0.0067092916 * Limelight::GetInstance().getVerticalOffset() + .9497390001 < .83){
            value = .83;
        } else {
            value = (0.0067092916 * Limelight::GetInstance().getVerticalOffset() + .9497390001);
        }
        frc::SmartDashboard::PutNumber("value", value);
        if (Limelight::GetInstance().getID() ==  3 || Limelight::GetInstance().getID() == 4 || Limelight::GetInstance().getID() == 7 || Limelight::GetInstance().getID() == 8){
            if (m_ClawPotentiometer2->Get() > value + .025 || m_ClawPotentiometer1->Get() > value + .025) {
                if (m_ClawPotentiometer2->Get() < value){
                    m_ClawActuator1->Set(0);
                    m_ClawActuator2->Set(1);
                } else if (m_ClawPotentiometer1->Get() < value){
                    m_ClawActuator1->Set(1);
                    m_ClawActuator2->Set(0);
                } else {
                    m_ClawActuator1->Set(1);
                    m_ClawActuator2->Set(1);
                }
                
            } else if (m_ClawPotentiometer2->Get() < value - .025 || m_ClawPotentiometer1->Get() < value - .025) {
                if (m_ClawPotentiometer2->Get() > value){
                    m_ClawActuator1->Set(0);
                    m_ClawActuator2->Set(-1);
                } else if (m_ClawPotentiometer1->Get() > value){
                    m_ClawActuator1->Set(-1);
                    m_ClawActuator2->Set(0);
                } else {
                    m_ClawActuator1->Set(-1);
                    m_ClawActuator2->Set(-1);
                }
            } else {
                m_ClawActuator1->Set(0);
                m_ClawActuator2->Set(0);
            }
        } else {
            if (m_ClawPotentiometer2->Get() > 0.82 || m_ClawPotentiometer1->Get() > .82) {
                if (m_ClawPotentiometer2->Get() < .805){
                    m_ClawActuator1->Set(0);
                    m_ClawActuator2->Set(1);
                } else if (m_ClawPotentiometer1->Get() < .805){
                    m_ClawActuator1->Set(1);
                    m_ClawActuator2->Set(0);
                } else {
                    m_ClawActuator1->Set(1);
                    m_ClawActuator2->Set(1);
                }
                
            } else if (m_ClawPotentiometer2->Get() < 0.79 || m_ClawPotentiometer1->Get() < .79) {
                if (m_ClawPotentiometer2->Get() > .805){
                    m_ClawActuator1->Set(0);
                    m_ClawActuator2->Set(-1);
                } else if (m_ClawPotentiometer1->Get() > .805){
                    m_ClawActuator1->Set(-1);
                    m_ClawActuator2->Set(0);
                } else {
                    m_ClawActuator1->Set(-1);
                    m_ClawActuator2->Set(-1);
                }
            } else {
                m_ClawActuator1->Set(0);
                m_ClawActuator2->Set(0);
            }           
        }

   
    }
    void SetAngle() {
        if (m_ClawPotentiometer2->Get() > 0.82 || m_ClawPotentiometer1->Get() > .82) {
            if (m_ClawPotentiometer2->Get() < .805){
                m_ClawActuator1->Set(0);
                m_ClawActuator2->Set(1);
            } else if (m_ClawPotentiometer1->Get() < .805){
                m_ClawActuator1->Set(1);
                m_ClawActuator2->Set(0);
            } else {
                m_ClawActuator1->Set(1);
                m_ClawActuator2->Set(1);
            }
                
        } else if (m_ClawPotentiometer2->Get() < 0.79 || m_ClawPotentiometer1->Get() < .79) {
            if (m_ClawPotentiometer2->Get() > .805){
                m_ClawActuator1->Set(0);
                m_ClawActuator2->Set(-1);
            } else if (m_ClawPotentiometer1->Get() > .805){
                m_ClawActuator1->Set(-1);
                m_ClawActuator2->Set(0);
            } else {
                m_ClawActuator1->Set(-1);
                m_ClawActuator2->Set(-1);
            }
        } else {
            m_ClawActuator1->Set(0);
            m_ClawActuator2->Set(0);
        } 
    }
    void TrackNew(){
        frc::SmartDashboard::PutNumber("Top", m_ClawPotentiometer1->Get());
        frc::SmartDashboard::PutNumber("Bottom", m_ClawPotentiometer2->Get());
        if (0.0067092916 * Limelight::GetInstance().getVerticalOffset() + .9137390001 > .89){
            value = .92;
        } else if (0.0067092916 * Limelight::GetInstance().getVerticalOffset() + .9197390001 < .79){
            value = .83;
        } else {
            value = (0.0067092916 * Limelight::GetInstance().getVerticalOffset() + .9197390001);
        }
        frc::SmartDashboard::PutNumber("value", value);
        if (Limelight::GetInstance().getID() ==  3 || Limelight::GetInstance().getID() == 4 || Limelight::GetInstance().getID() == 7 || Limelight::GetInstance().getID() == 8){
            if (m_ClawPotentiometer2->Get() > value + .025 || m_ClawPotentiometer1->Get() > value + .025) {
                if (m_ClawPotentiometer2->Get() < value){
                    m_ClawActuator1->Set(0);
                    m_ClawActuator2->Set(.5);
                } else if (m_ClawPotentiometer1->Get() < value){
                    m_ClawActuator1->Set(.5);
                    m_ClawActuator2->Set(0);
                } else {
                    m_ClawActuator1->Set(.5);
                    m_ClawActuator2->Set(.5);
                }
                Pow::GetInstance().Feed(0);
            } else if (m_ClawPotentiometer2->Get() < value - .025 || m_ClawPotentiometer1->Get() < value - .025) {
                if (m_ClawPotentiometer2->Get() > value){
                    m_ClawActuator1->Set(0);
                    m_ClawActuator2->Set(-.5);
                } else if (m_ClawPotentiometer1->Get() > value){
                    m_ClawActuator1->Set(-.5);
                    m_ClawActuator2->Set(0);
                } else {
                    m_ClawActuator1->Set(-.5);
                    m_ClawActuator2->Set(-.5);
                }
                Pow::GetInstance().Feed(0);
            } else {
                m_ClawActuator1->Set(0);
                m_ClawActuator2->Set(0);
                if (Limelight::GetInstance().getHorizontalOffset() < 3 || Limelight::GetInstance().getHorizontalOffset() > -3){
                    if (value > .79 && value < .89){
                        Pow::GetInstance().Feed(.5);
                        Intake::GetInstance().Speed1(.4);
                    } else {
                        Pow::GetInstance().Feed(0);
                    }        
                } else {
                    Pow::GetInstance().Feed(0);
                }
                
            }
        } else {
            m_ClawActuator1->Set(0);
            m_ClawActuator2->Set(0);
        }
    }
    private:
    Tests(const int R_ClawActuatorCANID1, const int R_ClawActuatorCANID2) {
        m_ClawActuator1 = new rev::CANSparkMax(R_ClawActuatorCANID1, rev::CANSparkMax::MotorType::kBrushed);
        m_ClawActuator2 = new rev::CANSparkMax(R_ClawActuatorCANID2, rev::CANSparkMax::MotorType::kBrushed);
        m_ClawPotentiometer1 = new frc::AnalogPotentiometer(1, 1.0, 0.0);
        m_ClawPotentiometer2 = new frc::AnalogPotentiometer(2, 1.0, 0.0);  
    }
    rev::CANSparkMax* m_ClawActuator1;
    rev::CANSparkMax* m_ClawActuator2;
    frc::AnalogPotentiometer* m_ClawPotentiometer1;
    frc::AnalogPotentiometer* m_ClawPotentiometer2;
    double value;

};
