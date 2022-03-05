// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.core751.wrappers.wAnalogPotentiometer;

public class Hood extends SubsystemBase {

  public enum BangBang{
    Idle,
    Updating
  }

  public double calibrationFactor; //The factor used for guessing the angle of the hood. 

  private VictorSPX hoodMotor;
  public double hoodAngle = 0;
  public double goalAngle;
  
  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;
  private wAnalogPotentiometer pot;

  private double softAngleMax = Constants.softAngleMax;
  private double softAngleMin = Constants.softAngleMin; //Just run into the bottom limit switch. 
  private double angleAdjustSpeed = Constants.angleAdjustSpeed;
  
  private BangBang state = BangBang.Idle;

  private boolean safeMode = true; 

  private double tolerance = Constants.angleTolerance;

  public Hood(int hoodMotorId, int topSwitchPin, int bottomSwitchPin,int potPin) {
    hoodMotor = new VictorSPX(hoodMotorId);
    hoodMotor.setInverted(false); //IDK yet.
    //this.topLimitSwitch = new DigitalInput(topSwitchPin);
    //this.bottomLimitSwitch = new DigitalInput(bottomSwitchPin);
    this.pot = new wAnalogPotentiometer(potPin,10,-535);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Pot Angle", pot.getAngle());
    SmartDashboard.putNumber("Pot Value", pot.get());
    // This method will be called once per scheduler run
  
    hoodAngle = pot.get() + calibrationFactor;

    //Making sure we don't over extend
    //Checking for contact every "frame" is there a better way?

    // if(this.topLimitSwitch.get()){
    //   adjustAngle(0);
    //   hoodAngle = Constants.hoodAngleMax;
    //   calibrationFactor = Constants.hoodAngleMax - pot.get();
    //  
    // }
    // else if(this.bottomLimitSwitch.get()){
    //   adjustAngle(0);
    //   hoodAngle = Constants.hoodAngleMin;
    //   calibrationFactor = Constants.hoodAngleMin - pot.get();
    // 
    //} 

    //For bang bang cycle.
    // if(state == BangBang.Updating){
    //   if (hoodAngle < goalAngle - Constants.angleTolerance || hoodAngle > goalAngle + Constants.angleTolerance){
    //     if(hoodAngle < goalAngle){
    //       adjustAngle(angleAdjustSpeed);
    //     }else if(hoodAngle > goalAngle){
    //       adjustAngle(-angleAdjustSpeed);
    //     }
    //   } else {
    //     adjustAngle(0);
    //     state = BangBang.Idle;
    //   }
    // }
  }

  public double normilizeSpeed(double speed){
    speed = Math.min(speed, 1);
    speed = Math.max(speed,-1);
    return speed;
  }

  /** Only use this if you KNOW what you are doing */
  public void setSafeMode(boolean safeMode){
    this.safeMode = safeMode;
  }
  

  //------------------Hood Functions------------------
  public void setAngle(double angle){ 
    angle = Math.min(angle, softAngleMax);
    angle = Math.max(angle, softAngleMin);

    this.goalAngle = angle;
    this.state = BangBang.Updating;

  }

  public void adjustAngle(double speed){
    speed = normilizeSpeed(speed);
    // if(this.topLimitSwitch.get()){
    //   speed = Math.min(0, speed);
    // }
    // else if(this.hoodAngle >= softAngleMax && this.safeMode){
    //   speed = Math.max(0, speed);
    // }
     //hoodMotor.set(speed);
  }

//------------------Getter Functions------------------
  public double getAngle(){
    return hoodAngle;
  }

  public BangBang getState(){
    return state;
  }

  public boolean getSaveMode(){
    return safeMode;
  }

  public double getCalibrationFactor(){
    return calibrationFactor;
  }

  public boolean atSetpoint(){
    return (this.hoodAngle <= this.goalAngle + this.tolerance) && (this.hoodAngle >= this.goalAngle - this.tolerance);
  }


}
