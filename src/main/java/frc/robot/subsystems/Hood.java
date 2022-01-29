// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {

  public enum BangBang{
    Idle,
    Updating
  }

  public double calibrationFactor; //The factor used for guessing the angle of the hood. 
  public AnalogPotentiometer potentiometer;

  private MotorController hoodMotor;
  public double hoodAngle = 0;
  public double goalAngle;
  
  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;
  private double softAngleMax = Constants.softAngleMax;
  private double softAngleMin = Constants.softAngleMin;
  private double angleAdjustSpeed = Constants.angleAdjustSpeed;
  
  private BangBang state = BangBang.Idle;

  private boolean safeMode = true; 

  public Hood(int topSwitchPin, int bottomSwitchPin) {

    this.topLimitSwitch = new DigitalInput(topSwitchPin);
    this.bottomLimitSwitch = new DigitalInput(bottomSwitchPin);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    hoodAngle = potentiometer.get() + calibrationFactor;

    //Making sure we don't over extend
    //Checking for contact every "frame" is there a better way?
    if(this.topLimitSwitch.get()){
      adjustAngle(0);
      hoodAngle = Constants.hoodAngleMax;
      calibrationFactor = Constants.hoodAngleMax - potentiometer.get();
      setAngle(softAngleMax);
    }
    else if(this.bottomLimitSwitch.get()){
      adjustAngle(0);
      hoodAngle = Constants.hoodAngleMin;
      calibrationFactor = Constants.hoodAngleMin - potentiometer.get();
      setAngle(softAngleMin);
    } 

    //For bang bang cycle.
    if(state == BangBang.Updating){
      if (hoodAngle < goalAngle - Constants.angleTolerance || hoodAngle > goalAngle + Constants.angleTolerance){
        if(hoodAngle < goalAngle){
          adjustAngle(angleAdjustSpeed);
        }else if(hoodAngle > goalAngle){
          adjustAngle(-angleAdjustSpeed);
        }
      } else {
        adjustAngle(0);
        state = BangBang.Idle;
      }
    }
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
    angle = Math.max(angle,softAngleMin);

    this.goalAngle = angle;
    this.state = BangBang.Updating;

  }

  public void adjustAngle(double speed){
    speed = normilizeSpeed(speed);
    if(this.hoodAngle >= softAngleMax && this.safeMode){
      speed = Math.min(0, speed);
    }
    else if(this.hoodAngle <= softAngleMin && this.safeMode){
      speed = Math.max(0, speed);
    }
     hoodMotor.set(speed);
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


}
