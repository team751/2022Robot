// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

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
    idle (Constants.shooterIdleSpeed),
    spinUp (Constants.spinUpSpeed);

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

  public float hoodAngle = Constants.hoodAngle;
  public float loadSpeed = Constants.loadSpeed;

  public static State currentState = State.idle;

  public static MotorController loadingMotor;
  public static MotorController hoodMotor; 
  public static CANSparkMax shootingMotor;

  public Shooter(MotorController loadingMotor, MotorController hoodMotor, CANSparkMax shootingMotor) {
    this.loadingMotor = loadingMotor;
    this.hoodMotor = hoodMotor;
    this.shootingMotor = shootingMotor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void idle(float speed){
    State.idle.setSpeed(speed);
    idle();
  }

  public void idle(){
    loadingMotor.set(State.idle.getSpeed());
    this.currentState = State.idle;
  } 

  /**
   * ENSURE THAT THE ROBOT IS IN SPIN UP BEFORE SHOOTING USING getState()
   */
  public void shoot(){
    if(currentState != State.spinUp){
      System.out.println("/\\/\\/\\/\\/\\/\\/\\/\\ \n Make sure to spin up Wheel!! \n /\\/\\/\\/\\/\\/\\/\\/\\");
      //Maybe throw an error?
    }
    loadingMotor.set(loadSpeed);
  }


  public void spinUp(float speed){
    State.spinUp.setSpeed(speed);
    spinUp();
  }

  public void spinUp(){ 
        shootingMotor.set(State.spinUp.getSpeed());
        this.currentState = State.spinUp;
  }
  
  //TODO make work
  public void angle(float angle){
    hoodAngle = angle;
  }

  //TODO Add limits
  public void adjustAngle(float speed){
     hoodMotor.set(speed);
  }

  public float getAngle(){
    return hoodAngle;
  }
  public State getState(){
    return currentState;
  }

}
//TODO
/*
  Add an angle calibration command to make sure we get our angle right on the hood. 

*/
