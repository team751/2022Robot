// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  
  /**
   * The enum for containing the state of the shooter.
   * Either Idle, or Spin Up
   * Make sure that the wheel is spining up before shooting.
   */
  public enum State {
    Idle (Constants.shooterIdleSpeed),
    SpinUp (Constants.spinUpSpeed);

    private float speed;
    private State(float speed){
      this.speed = speed;
    }
    public float getSpeed(){
      return this.speed;
    }
    public void setSpeed(float speed){
      this.speed = speed;
    }
}

  public enum Position{
    Top,
    Bottom,
    Neither;
  }

  public float calibrationFactor; //The factor used for guessing the angle of the hood. 

  public float hoodAngle = 0;
  public float loadSpeed = Constants.loadSpeed;

  public State currentState = State.Idle;
  public Position  currentPosition = Position.Neither;

  private MotorController loadingMotor;
  private MotorController hoodMotor; 
  private CANSparkMax shootingMotor;

  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;

  public Shooter(MotorController loadingMotor, MotorController hoodMotor, CANSparkMax shootingMotor,int topSwitchPin, int bottomSwitchPin) {
    this.loadingMotor = loadingMotor;
    this.hoodMotor = hoodMotor;
    this.shootingMotor = shootingMotor;

    this.topLimitSwitch = new DigitalInput(topSwitchPin);
    this.bottomLimitSwitch = new DigitalInput(bottomSwitchPin);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Making sure we don't over extend
    //Checking for contact every "frame" is there a better way?
    if(this.topLimitSwitch.get()){
      adjustAngle(0);
      hoodAngle = Constants.hoodAngleMax;
      currentPosition = Position.Top;
    }
    else if(this.bottomLimitSwitch.get()){
      adjustAngle(0);
      hoodAngle = Constants.hoodAngleMin;
      currentPosition = Position.Bottom;
    } else {
      currentPosition = Position.Neither;
    }
  }

  public void idle(float speed){
    State.Idle.setSpeed(speed);
    idle();
  }

  public void idle(){
    loadingMotor.set(State.Idle.getSpeed());
    this.currentState = State.Idle;
  } 

  //------------------Shooter Functions------------------
  /**
   * ENSURE THAT THE ROBOT IS IN SPIN UP BEFORE SHOOTING USING getState()
   */
  public void shoot(){
    if(currentState != State.SpinUp){
      System.out.println("/\\/\\/\\/\\/\\/\\/\\/\\ \n Make sure to spin up Wheel!! \n /\\/\\/\\/\\/\\/\\/\\/\\");
      //Maybe throw an error?
    }
    loadingMotor.set(loadSpeed);
  }


  public void spinUp(float speed){
    State.SpinUp.setSpeed(speed);
    spinUp();
  }

  public void spinUp(){ 
        shootingMotor.set(State.SpinUp.getSpeed());
        this.currentState = State.SpinUp;
  }

  //------------------Hood Functions------------------
  //TODO make work
  public void angle(float angle){
    hoodAngle = angle;
  }

  public void adjustAngle(float speed){
    if(this.currentPosition == Position.Top){
      speed = Math.min(0, speed);
    }
    else if(this.currentPosition == Position.Bottom){
      speed = Math.max(0, speed);
    }
     hoodMotor.set(speed);
  }

  //------------------Getter Functions------------------
  public float getAngle(){
    return hoodAngle;
  }
  public State getState(){
    return currentState;
  }
  public Position getPosition() {
      return currentPosition;
  }

}
//TODO
/*
  Add an angle calibration command to make sure we get our angle right on the hood. 

*/
