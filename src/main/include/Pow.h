#pragma once

#include "Robot.h"
#include <rev/CANSparkMax.h>
#include "math.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "commonauto/AutoStep.h"
#include <rev/CANSparkMaxLowLevel.h> // Include the REV library
#include "Consts.h"

class Pow {

public:
    static Pow& GetInstance() {
        static Pow* instance = new Pow(R_ShooterID, R_GrabberID);
        return *instance;
    }

    void Shoot(double speed) {
        shooter->Set(speed);
    }
    void Feed(double speed){
        grabber->Set(speed);
    }
    void AMP() {
        shooter->Set(-.2); // Adjust this value as needed
        grabber->Set(.4); // Adjust this value as needed
    }
    void FeedStop(){
        grabber->Set(0);
    }
    void Stop() {
        shooter->Set(0);
        grabber->Set(0); // Adjust this value as needed
    }

private:
    Pow(const int R_ShooterID, const int R_GrabberID) {
        shooter = new rev::CANSparkMax(R_ShooterID, rev::CANSparkMax::MotorType::kBrushless);
        grabber = new rev::CANSparkMax(R_GrabberID, rev::CANSparkMax::MotorType::kBrushless);
    }
    //rev::CANSparkMax shooter{R_ShooterID, rev::CANSparkMax::MotorType::kBrushless};
    //rev::CANSparkMax grabber{R_GrabberID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax* shooter;
    rev::CANSparkMax* grabber;
};
/*#pragma once

#include "Robot.h"
#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "commonauto/AutoStep.h"
#include <rev/CANSparkMaxLowLevel.h> // Include the REV library
#include "Consts.h"

class Pow {

public:
    static Pow& GetInstance() {
        static Pow instance(R_ShooterID, R_GrabberID);
        return instance;
    }

    void Shoot(double speed) {
        shooter->Set(speed);
    }

    void Feed(double speed){
        grabber->Set(speed);
    }

    void Source() {
        shooter->Set(-1); // Adjust this value as needed
        grabber->Set(0.4); // Adjust this value as needed
    }

    void FeedStop(){
        grabber->Set(0);
    }

    void Stop() {
        shooter->Set(0);
        grabber->Set(0); // Adjust this value as needed
    }

private:
    Pow(const int R_ShooterID, const int R_GrabberID) {
        shooter = new rev::CANSparkMax(R_ShooterID, rev::CANSparkMax::MotorType::kBrushless);
        grabber = new rev::CANSparkMax(R_GrabberID, rev::CANSparkMax::MotorType::kBrushless);
    }

    ~Pow() {
        delete shooter;
        delete grabber;
    }

    rev::CANSparkMax* shooter = nullptr;
    rev::CANSparkMax* grabber = nullptr;

    // Prevent copying and assignment
    Pow(const Pow&) = delete;
    Pow& operator=(const Pow&) = delete;
};*/
