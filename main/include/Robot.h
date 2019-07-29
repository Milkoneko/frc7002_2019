/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#pragma once
#include <string>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/Joystick.h>
#include <frc/VictorSP.h>
#include <cameraserver/CameraServer.h>
#include <frc/DriverStation.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/Spark.h>
#include <frc/DoubleSolenoid.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/Servo.h>

class Robot : public frc::TimedRobot {
 public:
 //函数定义
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void BaseControl();
  void StructureControl();

 private:
//dashboard
  frc::SmartDashboard *smartDashboard;
//马达定义
  frc::VictorSP LeftMotor{0};   //底盘
  frc::VictorSP RightMotor{1};  //底盘
  frc::VictorSP LiftMotor1{4};  //抬升
  frc::VictorSP LiftMotor2{3};  //抬升
  frc::VictorSP ForwardMotor{9};//抬升后前进
  frc::VictorSP JackMotor{6};   //机械臂上下抬升
  frc::VictorSP ExtendMotor{7}; //机械臂弧度伸展
  frc::VictorSP TongMotor{8};   //舱门夹
  frc::DifferentialDrive MotorDrive{LeftMotor,RightMotor};
  double driveSpeed=-1.0;
  double driveSpeedx=1.0;
//摇杆
  frc::XboxController stick1{0};
  frc::XboxController stick2{1};
//livewindow
  frc::LiveWindow& LWPinter = *frc::LiveWindow::GetInstance();
//计时器
  frc::Timer MyTimer;
  
//Driverstation
  frc::DriverStation& ds = frc::DriverStation::GetInstance();
  bool RobotMode;
//气动
  frc::Compressor *cp = new frc::Compressor(0); 
  bool CompressorState1 = cp->Enabled();
  bool PressureSwitch1 = cp->GetPressureSwitchValue();
  double CompressorCurrent1 = cp->GetCompressorCurrent();
//电磁气动阀声明
  frc::Solenoid SSolenoid {0};
  frc::DoubleSolenoid DSolenoid {1,2};
//servo
  frc::Servo *Servo1 = new frc::Servo(2); 
  int ServoTime=0;
//各种变量初始化
  bool BatteryState;
  double TongS,Lift1S,Lift2S;
  cs::UsbCamera camera{}; 
  bool BallCatch = false;
};
