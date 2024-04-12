#pragma once

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <cmath>  // Use cmath for abs function
#include <frc/smartdashboard/SmartDashboard.h>
#include "LimelightConsts.h"

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <string.h>
// Class definition for the Limelight
class Limelight {
public:
    // Singleton pattern implementation: GetInstance() returns a reference to the single instance of Limelight.
    static Limelight& GetInstance() {
        static Limelight instance;
        return instance;
    }
    // Accessor method to retrieve a double entry from the Limelight NetworkTable
    double getDoubleEntry(const std::string& entry) {
        
        return table->GetEntry(entry).GetDouble(0.0);
    }
    
    // Accessor method to retrieve the horizontal offset of the target
    double getHorizontalOffset() {
        const std::string horizontalKey = "tx";
        double offset = table->GetEntry(horizontalKey).GetDouble(0.0);
        frc::SmartDashboard::PutNumber("Horizontal Offset", offset);
        return offset;
    }
    
    // Accessor method to retrieve the vertical offset of the target
    double getVerticalOffset() {
        const std::string verticalKey = "ty";
        double offset = table->GetEntry(verticalKey).GetDouble(0.0);
        frc::SmartDashboard::PutNumber("Vertical Offset", offset);
        return offset;
    }
    double getBotPose() {
        const std::string Pose = "botpose";
        double pose = table->GetEntry(Pose).GetDouble(0.0);
        frc::SmartDashboard::PutNumber("Pose", pose);
        return pose;
    }
    double RobotTargetPose() {
        const std::string Target = "targetpose_robotspace";
        double target = table->GetEntry(Target).GetDouble(0.0);
        frc::SmartDashboard::PutNumber("Robot Target Pose", target);
        return target;
    }
    double CameraTargetPose () {
        const std::string Target = "camerapose_robotspace";
        double target = table->GetEntry(Target).GetDouble(0.0);
        frc::SmartDashboard::PutNumber("Camera Target Pose", target);
        return target;
    }
    std::string object() {
        const std::string item = "tclass";
        std::string name = table->GetEntry(item).GetString("none");
        frc::SmartDashboard::PutString("Object", name);
        return name;
    }
    // Accessor method to retrieve the target ID
    double getID() {
        const std::string Number = "tid";
        double ID = table->GetEntry(Number).GetDouble(0.0);
        frc::SmartDashboard::PutNumber("ID", ID);
        return ID;
        //return getDoubleEntry("tid");
    }

    
    // Accessor method to retrieve the target area
    double getTargetArea() {
        const std::string areaKey = "ta";
        return table->GetEntry(areaKey).GetDouble(0.0);
    }
    
    // Accessor method to check if a target is detected
    bool getTarget() {
        const std::string targetKey = "tv";
        bool hasTarget = table->GetEntry(targetKey).GetDouble(0.0) != 0.0;

        // Check if the target is an April Tag (ID 0) or reflective tape (ID 1)
        if (hasTarget) {
            double targetID = getID();
            hasTarget = (targetID == 0.0 || targetID == 1.0);
        }

        return hasTarget;
    }
    void Auto() {
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-ja")->PutNumber("pipeline", 0);  
        Limelight::GetInstance().setLime(false);       
        Limelight::GetInstance().setProcessing(true);         
    }
    void Reflective() {
        // Switch to Reflective Tape mode (assuming Reflective Tape pipeline is configured as pipeline 0)
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-ja")->PutNumber("pipeline", 0);
        Limelight::GetInstance().setLime(true);       
        Limelight::GetInstance().setProcessing(true);         
    }
    void April() {

        // Switch to April Tags mode (assuming April Tags pipeline is configured as pipeline )
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-ja")->PutNumber("pipeline", 3);
        Limelight::GetInstance().setLime(false);
    }
    void Items() {
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-ja")->PutNumber("pipeline", 4);
        Limelight::GetInstance().setLime(false);    
    }
    // Check if the horizontal offset is within the defined tolerance
    bool isWithinHorizontalTolerance() {
        return std::abs(getHorizontalOffset()) < R_zionAutoToleranceHorizontalOffset;
    }

    // Turn on or off vision processing for using the Limelight as a camera. Defaults to on.
    void setProcessing(const bool& toSet = true) {
        // According to doc, 1 is off, 0 is on
        table->PutNumber("camMode", toSet ? 0 : 1);
    }

    // Turn the Limelight LEDs on or off. Defaults to on.
    void setLime(const bool& toSet = true) {
        // According to doc, 3 is on, 1 is off, and 2 is blink
        table->PutNumber("ledMode", toSet ? 3 : 1);
    }
    // Calculate the rotational speed needed to keep the robot aligned with the detected target using data from the Limelight.


private:
    // Private constructor to enforce the singleton pattern.
    bool currentVisionMode{true};
    Limelight() {
        table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-ja");
    }

    // Deleted copy and move constructors and assignments to enforce singleton pattern.
    Limelight(const Limelight&) = delete;
    Limelight& operator=(const Limelight&) = delete;
    Limelight(Limelight&&) = delete;
    Limelight& operator=(Limelight&&) = delete;

    // Shared pointer to the Limelight NetworkTable.
    std::shared_ptr<nt::NetworkTable> table;

};
