// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
//#include "Claw.h"
#include "Intake.h"
#include "April.h"
#include "AMP.h"
#include "swerve/src/include/SwerveModule.h"
#include <math.h>
#include <frc/AnalogInput.h>
#include <fmt/core.h> 
#include <frc/Ultrasonic.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/XboxController.h>
#include <cameraserver/CameraServer.h>
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include "limelight/Limelight.h"
#include "limelight/LimelightBack.h"
#include "swerve/src/include/SwerveTrain.h"
#include "controller/Controller.h"
#include "commonauto/AutoSequence.h"
#include "commonauto/steps/WaitSeconds.h"
#include "commonauto/steps/TimeDriveHold.h"
#include "commonauto/steps/TurnToAbsoluteAngle.h"
#include "commonauto/steps/Stop.h"
#include "commonauto/steps/ResetNavXYaw.h"
#include "commonauto/steps/CalibrateNavXThenReset.h"
#include "commonauto/steps/MoveAndTurn.h"
#include "commonauto/steps/MoveWithoutStop.h"
#include "commonauto/steps/MoveAndTurnNew.h"
#include "Auto/FindPiece.h"
#include "Auto/PickUp.h"
#include "Auto/StopIntake.h"
#include "Auto/Shooter.h"
#include "Auto/SetArm.h"
#include "Auto/SetClaw.h"
#include "Auto/SetClaw2.h"
#include "Auto/ClimberA.h"
#include "Pow.h"
#include "Auto/Feed2.h"
#include "Climber.h"
#include "Item.h"
#include "Auto/Feed.h"
#include "Auto/SetClaw3.h"
#include "Auto/StopShooter.h"
#include "Auto/Track.h"
#include "Auto/Feed3.h"
#include "Auto/Lineup.h"
#include "Tests.h"

frc::AnalogInput* sonarSensor;
frc::XboxController* playerOne;
frc::XboxController* playerTwo;
AutoSequence* bigSequence;
frc::SendableChooser<std::string>* autoChooser;
frc::SendableChooser<std::string>* team;
frc::SendableChooser<std::string>* driver;
frc::AnalogPotentiometer *m_ClawPotentiometer;
double speed;
void Robot::RobotInit() {
NavX::GetInstance().Calibrate();
NavX::GetInstance().resetYaw();
playerOne = new frc::XboxController(R_controllerPortPlayerOne);
playerTwo = new frc::XboxController(R_controllerPortPlayerTwo);
sonarSensor = new frc::AnalogInput(0);
frc::CameraServer::StartAutomaticCapture();
LimelightBack::GetInstance().Normal();
LimelightBack::GetInstance().setLime(false);
Limelight::GetInstance().April();
autoChooser = new frc::SendableChooser<std::string>;
team = new frc::SendableChooser<std::string>;
driver = new frc::SendableChooser<std::string>;
SwerveTrain::GetInstance().HardwareZero();
autoChooser->AddOption("move backwards", "move backwards");
autoChooser->AddOption("mid right", "mid right");
autoChooser->AddOption("LeftBlue", "LeftBlue");
autoChooser->AddOption("MiddleBlue", "MiddleBlue");
autoChooser->AddOption("RightBlue", "RightBlue");
autoChooser->AddOption("LeftRed", "LeftRed");
autoChooser->AddOption("MiddleRed", "MiddleRed");
autoChooser->AddOption("RightRed", "RightRed");
autoChooser->AddOption("LeftBlueScoreThree", "LeftBlueScoreThree");
autoChooser->AddOption("test", "test");
autoChooser->AddOption("Shoot", "Shoot");
autoChooser->AddOption("NewRightBlue", "NewRightBlue");
autoChooser->AddOption("NewMidBlue", "NewMidBlue");
team->AddOption("red", "red");
team->AddOption("blue", "blue");
driver->AddOption("Braylon", "Braylon");
driver->AddOption("Other", "Other");
frc::SmartDashboard::PutData(autoChooser);
frc::SmartDashboard::PutData(team);
frc::SmartDashboard::PutData(driver);
bigSequence = new AutoSequence(false);
bigSequence->EnableLogging();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
    LimelightBack::GetInstance().Normal();
    LimelightBack::GetInstance().setProcessing(true);
    LimelightBack::GetInstance().setLime(false);
    NavX::GetInstance().Calibrate();
    NavX::GetInstance().resetYaw();
    SwerveTrain::GetInstance().SetSwerveBrake(true);
    SwerveTrain::GetInstance().SetDriveBrake(true);
    //SwerveTrain::GetInstance().ResetHold();
    //SwerveTrain::GetInstance().HardwareZero();
    bigSequence->Reset();
    Limelight::GetInstance().setProcessing(true);
    Limelight::GetInstance().Auto();

    std::string selectedAuto = autoChooser->GetSelected();
    //Braylon's auto have been changed recently, ask what the auto does
    //and where it should start because it has been changed since of 4/15/23
    if (selectedAuto == "move backwards") {
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new WaitSeconds(1));
      bigSequence->AddStep(new ResetNavXYaw);
      bigSequence->AddStep(new TimeDriveHold(0, -0.8, 2.6, false));


    SwerveTrain::GetInstance().SetSwerveBrake(true);
    SwerveTrain::GetInstance().SetDriveBrake(true);

    bigSequence->AddStep(new WaitSeconds(20));
    }
    else if(selectedAuto == "mid right"){
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new WaitSeconds(1));
      bigSequence->AddStep(new ResetNavXYaw);

      bigSequence->AddStep(new TimeDriveHold(-.75, 0, 2, false));
      bigSequence->AddStep(new TimeDriveHold(0, -.5, 3.5, false));
      bigSequence->AddStep(new TimeDriveHold(.75, 0, 2, false));

      SwerveTrain::GetInstance().SetSwerveBrake(true);
      SwerveTrain::GetInstance().SetDriveBrake(true);

      bigSequence->AddStep(new WaitSeconds(20));
    }

    else if (selectedAuto == "RightBlue"){
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new Shooter(.75));
      bigSequence->AddStep(new WaitSeconds(.9));
      bigSequence->AddStep(new ResetNavXYaw);
      bigSequence->AddStep(new Feed2(.7));
      bigSequence->AddStep(new WaitSeconds(.4));
      bigSequence->AddStep(new Shooter(0));
      bigSequence->AddStep(new Feed2(0));
      bigSequence->AddStep(new MoveAndTurnNew(0.5,-1,20,1,false,false,1));
      bigSequence->AddStep(new MoveWithoutStop(1,0,1,true,false,.9));
      bigSequence->AddStep(new PickUp());
      bigSequence->AddStep(new MoveAndTurnNew(0.15,-1,20,1.35,false,false,.8));
      bigSequence->AddStep(new StopIntake());
      bigSequence->AddStep(new TurnToAbsoluteAngle(20));
      bigSequence->AddStep(new WaitSeconds(.2));
      bigSequence->AddStep(new MoveAndTurnNew(-0.15,1,0,1.35,false,false,.8));
      bigSequence->AddStep(new MoveWithoutStop(-1,0,1,true,false,.92));
      bigSequence->AddStep(new Shooter(.69));
      bigSequence->AddStep(new MoveAndTurnNew(-0.5,1,5,.95,false,false,1.1));
      bigSequence->AddStep(new TurnToAbsoluteAngle(5));
      bigSequence->AddStep(new Feed2(.7));
      bigSequence->AddStep(new WaitSeconds(.65));
      bigSequence->AddStep(new Feed2(0));
      bigSequence->AddStep(new StopIntake());
      bigSequence->AddStep(new Shooter(0));
      /*bigSequence->AddStep(new MoveAndTurnNew(-1,.4,5,1.9,false,false,.95));
      bigSequence->AddStep(new Shooter(.75));
      bigSequence->AddStep(new MoveWithoutStop(0,1,1.4,true,false,.8));
      bigSequence->AddStep(new Feed(.7));
      bigSequence->AddStep(new TurnToAbsoluteAngle(5));
      bigSequence->AddStep(new WaitSeconds(.3));
      bigSequence->AddStep(new Feed(0));
      bigSequence->AddStep(new Shooter(0));*/
    } 
    else if (selectedAuto == "LeftRed"){
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new Shooter(.75));
      bigSequence->AddStep(new PickUp());
      bigSequence->AddStep(new WaitSeconds(.35));
      bigSequence->AddStep(new ResetNavXYaw);
      bigSequence->AddStep(new SetClaw(1));
      bigSequence->AddStep(new WaitSeconds(.7));
      bigSequence->AddStep(new SetClaw(0));
      bigSequence->AddStep(new Feed(.9));
      bigSequence->AddStep(new WaitSeconds(.3));
      bigSequence->AddStep(new Shooter(0));
      bigSequence->AddStep(new Feed(0));
      bigSequence->AddStep(new MoveAndTurnNew(-.85,-1,-40,1.6,false,false,.3));
      bigSequence->AddStep(new StopIntake());
      bigSequence->AddStep(new Shooter(.75));
      bigSequence->AddStep(new MoveAndTurnNew(.75,1,-10,1.8,false,false,.3));
      bigSequence->AddStep(new MoveWithoutStop(0.01,-.01,.3,false,false,0.01));
      bigSequence->AddStep(new Stop());
      bigSequence->AddStep(new Feed2(.7));
      bigSequence->AddStep(new WaitSeconds(.3));
      bigSequence->AddStep(new Feed2(0));
      bigSequence->AddStep(new Shooter(0));
      bigSequence->AddStep(new PickUp());
      bigSequence->AddStep(new MoveWithoutStop(-1,-1,1.9,false,false,1));
      bigSequence->AddStep(new PickUp());
      bigSequence->AddStep(new MoveWithoutStop(0,-.5,1.25,false,false,.4));
      bigSequence->AddStep(new WaitSeconds(.2));
      bigSequence->AddStep(new StopIntake());
      bigSequence->AddStep(new MoveWithoutStop(1, 1, 1.98, false, false, 1));
      bigSequence->AddStep(new Shooter(.75));
      bigSequence->AddStep(new MoveWithoutStop(0, 1, .8, false, false, .4));
      bigSequence->AddStep(new MoveWithoutStop(.01,.01,.2,false,false,.01));
      bigSequence->AddStep(new Stop());
      bigSequence->AddStep(new Feed2(.9));
      bigSequence->AddStep(new WaitSeconds(.3));
      bigSequence->AddStep(new Feed2(0));
      bigSequence->AddStep(new Shooter(0));
    } 
    else if (selectedAuto == "MiddleBlue"){
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new Shooter(.75));
      bigSequence->AddStep(new PickUp());
      bigSequence->AddStep(new WaitSeconds(1.2));
      bigSequence->AddStep(new Feed2(.7));
      bigSequence->AddStep(new WaitSeconds(.3));
      bigSequence->AddStep(new Feed2(0));
      bigSequence->AddStep(new Shooter(0));
      bigSequence->AddStep(new PickUp());
      bigSequence->AddStep(new MoveWithoutStop(0.15,-1,1.1,false,false,.4));
      bigSequence->AddStep(new StopIntake());
      bigSequence->AddStep(new Shooter(.75));
      bigSequence->AddStep(new MoveWithoutStop(-0.05,1,1.22,false,false,.4));
      bigSequence->AddStep(new MoveWithoutStop(-0.01,-.01,.3,false,false,0.01));
      bigSequence->AddStep(new Stop());
      bigSequence->AddStep(new Feed2(.7));
      bigSequence->AddStep(new WaitSeconds(.3));
      bigSequence->AddStep(new Feed2(0));
      bigSequence->AddStep(new Shooter(0));
      bigSequence->AddStep(new TurnToAbsoluteAngle(180));
    }
    else if (selectedAuto == "LeftBlue"){
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new Shooter(.75));
      bigSequence->AddStep(new PickUp());
      bigSequence->AddStep(new WaitSeconds(1.2));
      bigSequence->AddStep(new ResetNavXYaw);
      /*bigSequence->AddStep(new SetClaw(1));
      bigSequence->AddStep(new WaitSeconds(.17));
      bigSequence->AddStep(new SetClaw(0));*/
      bigSequence->AddStep(new Feed2(.7));
      bigSequence->AddStep(new WaitSeconds(.3));
      bigSequence->AddStep(new Feed2(0));
      bigSequence->AddStep(new Shooter(0));
      bigSequence->AddStep(new StopIntake());
      bigSequence->AddStep(new TimeDriveHold(0,-1,2.2,false,false));
      bigSequence->AddStep(new WaitSeconds(.1));
      bigSequence->AddStep(new TurnToAbsoluteAngle(135));
      /*bigSequence->AddStep(new MoveWithoutStop(1,-1,.85,false,false,.7));
      bigSequence->AddStep(new PickUp());
      bigSequence->AddStep(new MoveAndTurnNew(-1,-.85,270,2.55,false,false,.6));
      bigSequence->AddStep(new MoveWithoutStop(-0.01,-.01,.3,false,false,0.01));
      bigSequence->AddStep(new MoveWithoutStop(-1,.65,.85,false,false,.6));
      bigSequence->AddStep(new MoveWithoutStop(-0.01,-.01,.3,false,false,0.01));
      bigSequence->AddStep(new Stop());
      bigSequence->AddStep(new StopIntake());*/
    }
        else if (selectedAuto == "MiddleRed"){
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new Shooter(.7));
      bigSequence->AddStep(new PickUp());
      bigSequence->AddStep(new WaitSeconds(.5));
      bigSequence->AddStep(new Feed2(.7));
      bigSequence->AddStep(new WaitSeconds(.3));
      bigSequence->AddStep(new Feed2(0));
      bigSequence->AddStep(new Shooter(0));
      bigSequence->AddStep(new PickUp());
      bigSequence->AddStep(new WaitSeconds(.5));
      bigSequence->AddStep(new MoveWithoutStop(0.15,-1,1.25,false,false,.4));
      bigSequence->AddStep(new StopIntake());
      bigSequence->AddStep(new Shooter(.75));
      bigSequence->AddStep(new MoveWithoutStop(-0.05,1,1.37,false,false,.4));
      bigSequence->AddStep(new MoveWithoutStop(-0.01,-.01,.3,false,false,0.01));
      bigSequence->AddStep(new Stop());
      bigSequence->AddStep(new TurnToAbsoluteAngle(5));
      bigSequence->AddStep(new Feed2(.7));
      bigSequence->AddStep(new WaitSeconds(.3));
      bigSequence->AddStep(new Feed2(0));
      bigSequence->AddStep(new Shooter(0));
      bigSequence->AddStep(new TurnToAbsoluteAngle(180));
    }
    else if (selectedAuto == "RightRed"){
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new Shooter(.75));
      bigSequence->AddStep(new PickUp());
      bigSequence->AddStep(new WaitSeconds(1.2));
      bigSequence->AddStep(new ResetNavXYaw);
      /*bigSequence->AddStep(new SetClaw(1));
      bigSequence->AddStep(new WaitSeconds(.17));
      bigSequence->AddStep(new SetClaw(0));*/
      bigSequence->AddStep(new Feed2(.7));
      bigSequence->AddStep(new WaitSeconds(.3));
      bigSequence->AddStep(new Feed2(0));
      bigSequence->AddStep(new Shooter(0));
      bigSequence->AddStep(new StopIntake());
      bigSequence->AddStep(new TimeDriveHold(0.2,-1,2.2,false,false));
      bigSequence->AddStep(new WaitSeconds(.1));
      bigSequence->AddStep(new TurnToAbsoluteAngle(225));
      /*bigSequence->AddStep(new MoveWithoutStop(1,-1,.85,false,false,.7));
      bigSequence->AddStep(new PickUp());
      bigSequence->AddStep(new MoveAndTurnNew(-1,-.85,270,2.55,false,false,.6));
      bigSequence->AddStep(new MoveWithoutStop(-0.01,-.01,.3,false,false,0.01));
      bigSequence->AddStep(new MoveWithoutStop(-1,.65,.85,false,false,.6));
      bigSequence->AddStep(new MoveWithoutStop(-0.01,-.01,.3,false,false,0.01));
      bigSequence->AddStep(new Stop());
      bigSequence->AddStep(new StopIntake());*/
    }
    else if (selectedAuto == "Shoot"){
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new Shooter(.75));
      bigSequence->AddStep(new WaitSeconds(1.2));
      bigSequence->AddStep(new Feed2(.7));
      bigSequence->AddStep(new WaitSeconds(.3));
      bigSequence->AddStep(new Feed2(0));
      bigSequence->AddStep(new Shooter(0)); 
    }
    else if (selectedAuto == "LeftBlueScoreThree"){
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new Shooter(.7));
      bigSequence->AddStep(new WaitSeconds(.5));
      bigSequence->AddStep(new ResetNavXYaw);
      bigSequence->AddStep(new Feed2(.7));
      bigSequence->AddStep(new MoveWithoutStop(0,-1,1.2,false,false,.7));
      bigSequence->AddStep(new MoveAndTurnNew(-.7,-.7,10,1,false,false,.7));
      bigSequence->AddStep(new PickUp());
      bigSequence->AddStep(new Track());
    }
    else if (selectedAuto == "test"){
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new WaitSeconds(.5));
      bigSequence->AddStep(new ResetNavXYaw); 
      bigSequence->AddStep(new Shooter(.65));    
      bigSequence->AddStep(new Lineup());
      bigSequence->AddStep(new Feed3(.6));
      bigSequence->AddStep(new WaitSeconds(.3));
      bigSequence->AddStep(new Shooter(0));
      bigSequence->AddStep(new Feed3(0));
    }
    else if (selectedAuto == "NewMidBlue"){
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new Shooter(.7));
      bigSequence->AddStep(new WaitSeconds(.5));
      bigSequence->AddStep(new ResetNavXYaw);
      bigSequence->AddStep(new TimeDriveHold(0,-1,1.25,true,false));
      bigSequence->AddStep(new Feed2(.7));
      bigSequence->AddStep(new WaitSeconds(.3));
      bigSequence->AddStep(new Feed2(0));
      bigSequence->AddStep(new Shooter(0));
      bigSequence->AddStep(new PickUp());
      bigSequence->AddStep(new WaitSeconds(.3));
      bigSequence->AddStep(new MoveWithoutStop(0,-1,.75,true,false,.6));
      bigSequence->AddStep(new StopIntake());
      bigSequence->AddStep(new Shooter(.75));
      bigSequence->AddStep(new TimeDriveHold(0,1,1.75,true,false));
      bigSequence->AddStep(new Feed2(.75));
      bigSequence->AddStep(new WaitSeconds(.4));
      bigSequence->AddStep(new Shooter(0));
      bigSequence->AddStep(new Feed2(0));
      bigSequence->AddStep(new TurnToAbsoluteAngle(40));
      bigSequence->AddStep(new PickUp());
      bigSequence->AddStep(new MoveWithoutStop(0,-1,.735,true,false,1));
      bigSequence->AddStep(new TurnToAbsoluteAngle(40));
      bigSequence->AddStep(new WaitSeconds(.2));
      bigSequence->AddStep(new StopIntake());
      bigSequence->AddStep(new Shooter(.7));
      bigSequence->AddStep(new MoveWithoutStop(0,1,.735,true,false,1));
      bigSequence->AddStep(new TurnToAbsoluteAngle(0));
      bigSequence->AddStep(new Feed2(.7));
      bigSequence->AddStep(new WaitSeconds(.3));
      bigSequence->AddStep(new Shooter(0));
      bigSequence->AddStep(new Feed2(0));
    }
    else if (selectedAuto == "NewRightBlue"){
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new Shooter(.7));
      bigSequence->AddStep(new WaitSeconds(.5));
      bigSequence->AddStep(new ResetNavXYaw);
      bigSequence->AddStep(new Feed2(.7));
      bigSequence->AddStep(new WaitSeconds(.3));
      bigSequence->AddStep(new TimeDriveHold(0.4,-1,1.4,true));
      bigSequence->AddStep(new Feed2(0));
      bigSequence->AddStep(new Shooter(0));
      bigSequence->AddStep(new TimeDriveHold(1,0,1.6,true));
      bigSequence->AddStep(new PickUp());
      bigSequence->AddStep(new MoveAndTurn(.15,-1,20,2.4,false,false));
    }

    bigSequence->AddStep(new Stop);
    bigSequence->Init();


}
void Robot::AutonomousPeriodic() {
  LimelightBack::GetInstance().setLime(false);
  /* if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  } */
  bigSequence->Execute();
  
}

void Robot::TeleopInit() {
    // Set swerve brake to ensure stability
    SwerveTrain::GetInstance().SetSwerveBrake(true);
    // Set drive brake to ensure stability
    SwerveTrain::GetInstance().SetDriveBrake(true);
    // Set back Limelight camera mode to normal
    LimelightBack::GetInstance().Normal();
    // Enable processing for back Limelight camera
    LimelightBack::GetInstance().setProcessing(true);
    // Disable Lime mode for back Limelight camera
    LimelightBack::GetInstance().setLime(false);
    // Enable processing for front Limelight camera
    Limelight::GetInstance().setProcessing(true);
    // Set front Limelight camera to April tag mode
    Limelight::GetInstance().April();

    // Create a Pose2d object with arbitrary coordinates
    //frc::Pose2d robotPose(frc::Translation2d(0.0_m, 0.0_m), frc::Rotation2d());
    //NavX::GetInstance().setRobotOrientation(robotPose);    
    // Set the robot's orientation
}


void Robot::TeleopPeriodic() {
    // Update SmartDashboard with vertical offset from Limelight camera
    frc::SmartDashboard::PutNumber("Vertical", Limelight::GetInstance().getVerticalOffset());

    // Get selected team for control configuration
    std::string selectedTeam = team->GetSelected();
    // Get selected player (driver) for control configuration
    std::string selectedPlayer = driver->GetSelected();
    frc::SmartDashboard::PutString("driver", selectedPlayer);
    // Calibrate NavX gyro and reset yaw when left bumper is pressed
    if (playerOne->GetLeftBumper()) {
        NavX::GetInstance().Calibrate();
        NavX::GetInstance().resetYaw();
    }

    // Read controller input for drive controls and ensure deadzone correction
    double x = playerOne->GetLeftX();
    double y = playerOne->GetLeftY();
    double z = playerOne->GetRightX() * .75;
    Controller::forceControllerXYZToZeroInDeadzone(x, y, z);

    // Rumble feedback logic based on the current intake current
    frc::SmartDashboard::PutNumber("Current", Intake::GetInstance().Current());
    if (Intake::GetInstance().Current() > 14.5) {
        playerOne->SetRumble(frc::GenericHID::RumbleType::kBothRumble, .55);
        playerTwo->SetRumble(frc::GenericHID::RumbleType::kBothRumble, .55);
    } else {
        playerOne->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
        playerTwo->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
    }
    // Control logic for different player (driver) configurations based on the selected player
    if (selectedPlayer == "Braylon"){
      // Control logic based on button inputs from player one's controller
      if (playerOne->GetAButton()) {
          // Activate intake at a speed of 0.65
          Intake::GetInstance().Speed1(.65);
          // Check if the item tracking mode is disabled (returning 0 indicates no item found)
          if (Item::GetInstance().track() == 0) {
              // Drive the SwerveTrain using controller inputs (-x, -y, z) and default track speed
              SwerveTrain::GetInstance().Drive(-x, -y, z, false, false, 1);
          } else {
              // Drive the SwerveTrain using controller inputs (-x, -y, trackSpeed) with tracked speed
              SwerveTrain::GetInstance().Drive(-x, -y, Item::GetInstance().track(), false, false, 1);
          }
      } else if (playerOne->GetXButton()) {
          // Activate intake in reverse at a speed of -0.65
          Intake::GetInstance().Speed1(-.65);
          // Drive the SwerveTrain using controller inputs (-x, -y, z)
          SwerveTrain::GetInstance().Drive(-x, -y, z, false, false, 1);
      } else if (playerOne->GetRightTriggerAxis() > .25) {
          // Shoot at a speed of 70%
          Pow::GetInstance().Shoot(-.7);
          // Aim using the April tag system (horizontal)
          April::GetInstance().Aim();
          // Tracking the target for angle adjustment (Shooter)
          Tests::GetInstance().TrackNew();
      } else {
          // Drive the SwerveTrain using controller inputs (-x, -y, z)
          SwerveTrain::GetInstance().Drive(-x, -y, z, false, false, 1);
          // Deactivate intake
          Intake::GetInstance().Speed1(0);
          // Set the angle to low for clearance
          Tests::GetInstance().SetAngle(); 
          // Stop shooting
          Pow::GetInstance().Stop();
      }

      // Climber moves Up if right bumper is held then goes down on its own
      if (playerOne->GetRightBumper()) {
          // Move the climber up
          Climber::GetInstance().speed(-1);
      } else {
          // Move the climber down
          Climber::GetInstance().speed(1);
      }
    } else {
      // Aim Tracker
      Tests::GetInstance().TrackRed();
      if (playerOne->GetLeftTriggerAxis() > .15){
        April::GetInstance().Aim();
      } else {
        SwerveTrain::GetInstance().Drive(
          -x,
          -y,
          z,
          false,
          false,
          1
        );
      }
      if (playerTwo->GetAButton()) {
        Intake::GetInstance().Speed1(.65);
      } else if (playerTwo->GetXButton()){
          Intake::GetInstance().Speed1(-.65);
      } else {
        if (playerTwo->GetRightTriggerAxis() > .25){
          Pow::GetInstance().Shoot(-.7);
          if (playerTwo->GetYButton()){
            Pow::GetInstance().Feed(.65);
            Intake::GetInstance().Speed1(.7);
          } else {
            Pow::GetInstance().FeedStop();
            Intake::GetInstance().Speed1(0);
          }
        }
        else if (playerTwo->GetLeftTriggerAxis() > .25){
          Pow::GetInstance().AMP();
        } else {
          Pow::GetInstance().Stop();
          Intake::GetInstance().Speed1(0);
        }
      }
      if (playerTwo->GetRightBumper() || playerTwo->GetLeftBumper()){
        Climber::GetInstance().speed(-1);
      } else {
        Climber::GetInstance().speed(1);
      }
    }

      /*if (playerTwo->GetLeftBumper()){
        Climber::GetInstance().speed(1);
        Claw::GetInstance().Climb();
      } else if (playerTwo->GetRightBumper()){
        Climber::GetInstance().speed(-1);
      } else {
        Climber::GetInstance().speed(0);
        if (selectedTeam == "blue"){
          Claw::GetInstance().TrackBlue();
        } else if (selectedTeam == "red") {
          Claw::GetInstance().TrackRed();
        } 
        Claw::GetInstance().Update();
      }
      if (playerTwo->GetXButton()){
        Intake::GetInstance().Speed1(-.5);
      }
      */

      /*if (playerTwo->GetRightBumper()){
        Tests::GetInstance().ClawTilt(.5);
      } else if (playerTwo->GetLeftBumper()){
        Tests::GetInstance().ClawTilt(-.5);
      } else {
        Tests::GetInstance().ClawTilt(0);
      }*/


    //Speaker Shooter 
  

}




void Robot::DisabledInit() {
  SwerveTrain::GetInstance().SetSwerveBrake(false);
  SwerveTrain::GetInstance().SetDriveBrake(false);
}

void Robot::DisabledPeriodic() {
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
