/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*-------------------------------FRC7002--------------------------------------*/
/*-----------------------Coding in Shanxi with ❤-----------------------------*/
/*----------------------------------------------------------------------------*/
#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>
#include <frc/XboxController.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Servo.h>
#include <frc/RobotController.h>
#include <frc/MotorSafety.h>
#include <frc/BuiltInAccelerometer.h>

//机器人初始化--------------------------------------------------------------
void Robot::RobotInit() 
{
//速度初始化
  driveSpeed = -0.25;
  driveSpeedx = 0.25;
//摄像头
  camera=frc::CameraServer::GetInstance()->StartAutomaticCapture();
  camera.SetResolution(100,133);
  camera.SetExposureAuto();
  camera.SetFPS(18);
  camera.SetBrightness(20);
//马达安全
  MotorDrive.SetExpiration(0.1);
//气动初始化
  cp->SetClosedLoopControl(true); 
//舵机角度初始化
  Servo1 -> SetAngle(0);
//马达安全初始化
  LeftMotor.frc::MotorSafety::Feed();
  RightMotor.frc::MotorSafety::Feed();
}

//机器人周期执行--------------------------------------------------------------
void Robot::RobotPeriodic() 
{


//安全控制--------------------------
  //一号手柄摁下back，马达强制停止
    if(stick1.GetBackButton())
    {
    MotorDrive.StopMotor();
    }
  //二号手柄摁下back，电磁阀强制停止
    if(stick2.GetBackButton())
    {
    DSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
    SSolenoid.Set(false);
    }
  //自动停止超时电机
    LeftMotor.frc::MotorSafety::Check();
    RightMotor.frc::MotorSafety::Check();
    BatteryState = !frc::RobotController::IsBrownedOut();
//dashboard
  frc::SmartDashboard::PutBoolean("压缩机状态", CompressorState1);
  frc::SmartDashboard::PutBoolean("压力开关状态", PressureSwitch1);
  frc::SmartDashboard::PutNumber("马达转速",driveSpeedx);
  frc::SmartDashboard::PutNumber("气压", CompressorCurrent1);
  frc::SmartDashboard::PutBoolean("电池状态",BatteryState);
  frc::SmartDashboard::PutBoolean("左马达状态",LeftMotor.frc::MotorSafety::IsAlive());
  frc::SmartDashboard::PutBoolean("右马达状态",RightMotor.frc::MotorSafety::IsAlive());
  frc::SmartDashboard::PutNumber("当前电流",frc::RobotController::GetInputCurrent());
  frc::SmartDashboard::PutNumber("当前电压",frc::RobotController::GetInputVoltage());
}

//底盘控制部分--------------------------------------------------------------
void Robot::BaseControl()
{
  //手柄左摇杆控制机器底盘
    MotorDrive.CurvatureDrive(stick1.GetY(frc::XboxController::JoystickHand::kLeftHand)*driveSpeed, stick1.GetX(frc::XboxController::JoystickHand::kLeftHand)*0.7*driveSpeedx,true);
  //手柄右摇杆控制抬升后前进上台
    ForwardMotor.Set(stick1.GetY(frc::XboxController::JoystickHand::kRightHand));
  //摁下LB抬升后电机 B反转
    Lift1S = stick2.GetBumper(frc::XboxController::JoystickHand::kLeftHand) - stick2.GetBButton();
    LiftMotor1.Set(-Lift1S*0.90);
  //摁下RB抬升前电机 A反转
    Lift2S = stick2.GetBumper(frc::XboxController::JoystickHand::kRightHand) - stick2.GetAButton();
    LiftMotor2.Set(-Lift2S*0.85);
  //摁start切换舵机角度
    if (stick1.GetStartButtonPressed())
    {
      ServoTime++;
      if(ServoTime%2==0)
      {
        Servo1 -> SetAngle(100);
      }
      else
      {
        Servo1 -> SetAngle(0);
      }
      
    }
  //换挡
  //一档
    if(stick1.GetXButtonPressed())
    {
      driveSpeed = -0.25;
      driveSpeedx = 0.25;
    }
  //二档
    if(stick1.GetYButtonPressed())
    {
      driveSpeed = -0.4;
      driveSpeedx = 0.4;
    }
  //三档
    if(stick1.GetBButtonPressed())
    {
      driveSpeed = -0.75;
      driveSpeedx = 0.75;
    }
  //四档
    if(stick1.GetAButtonPressed())
    {
      driveSpeed = -1.0;
      driveSpeedx = 1.0;
    }
}
//上层控制部分--------------------------------------------------------------
void Robot::StructureControl()
{
//手柄2X松球
 if (stick2.GetXButtonPressed())
  {
    DSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    BallCatch = false;
  }
//手柄2Y夹球
 if (stick2.GetYButtonPressed())
  {
    DSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    BallCatch = true;
  }
//手柄2摁下start射球
  SSolenoid.Set(stick2.GetStartButton());
//↑↑↑气动部分-------电机部分↓↓↓
//手柄2左摇杆操控机械臂升降
  JackMotor.Set(stick2.GetY(frc::XboxController::JoystickHand::kLeftHand));
//手柄2右摇杆控制机械臂伸缩
  ExtendMotor.Set(-stick2.GetY(frc::XboxController::JoystickHand::kRightHand));
//手柄1 LB抓盘 RB松钩
  TongS = stick1.GetBumper(frc::XboxController::JoystickHand::kLeftHand) - stick1.GetBumper(frc::XboxController::JoystickHand::kRightHand);
  TongMotor.Set(TongS);
}
//自动初始化--------------------------------------------------------------
void Robot::AutonomousInit() {}
//自动周期执行--------------------------------------------------------------
void Robot::AutonomousPeriodic()
{
  Robot::StructureControl();
  Robot::BaseControl();
}

//远控初始化--------------------------------------------------------------
void Robot::TeleopInit() {}

//远控周期执行--------------------------------------------------------------
void Robot::TeleopPeriodic() 
{
  Robot::StructureControl();
  Robot::BaseControl();
}
//测试代码周期执行--------------------------------------------------------------
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

//------------------------------HIMAWARI🌻------------------------------------