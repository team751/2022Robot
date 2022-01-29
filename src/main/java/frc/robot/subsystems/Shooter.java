// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
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

    private double speed;
    private State(double speed){
      this.speed = speed;
    }
    public void setSpeed(double speed){
      this.speed = speed;
    }
    public double getSpeed(){
      return this.speed;
    }
    
}
  
  public float loadSpeed = Constants.loadSpeed;

  public State currentState = State.Idle;

  private MotorController loadingMotor;
  private CANSparkMax shootingMotor;


  public Shooter(MotorController loadingMotor, CANSparkMax shootingMotor) {
    this.loadingMotor = loadingMotor;
    this.shootingMotor = shootingMotor;    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double normilizeSpeed(double speed){
    speed = Math.min(speed, 1);
    speed = Math.max(speed,-1);
    return speed;
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

  public void spinUp(double speed){
    speed = normilizeSpeed(speed);
    State.SpinUp.setSpeed(speed);
    spinUp();
  }

  public void spinUp(){ 
    shootingMotor.set(State.SpinUp.getSpeed());
    this.currentState = State.SpinUp;
  }

  public void idle(double speed){
    speed = normilizeSpeed(speed);
    State.Idle.setSpeed(speed);
    idle();
  }

  public void idle(){
    loadingMotor.set(State.Idle.getSpeed());
    this.currentState = State.Idle;
  } 
  
  //------------------Getter Functions------------------

  public State getState(){
    return currentState;
  }

}
