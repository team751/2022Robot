// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.core751.wrappers.AnalogDistanceSensor;
import frc.robot.core751.wrappers.wTalonFX;
import frc.robot.core751.wrappers.AnalogDistanceSensor.SensorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterDeprecated extends SubsystemBase {
  
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

  private CANSparkMax loadingMotor;
  private wTalonFX shootingMotor;

  private AnalogDistanceSensor loadingSensor;
  private AnalogDistanceSensor holdingSensor;

  private double loadingDistance;
  private double holdingDistance;





  public ShooterDeprecated(int loadingMotorId, int shootingMotorId) {
    this.loadingMotor = new CANSparkMax(loadingMotorId,MotorType.kBrushless);
    this.shootingMotor = new wTalonFX(shootingMotorId);    
    //this.holdingSensor = new AnalogDistanceSensor(new AnalogInput(holdingSensorId),SensorType.MB1043);
    //this.loadingSensor = new AnalogDistanceSensor(new AnalogInput(loadingSensorId),SensorType.MB1043);

  }

  @Override
  public void periodic() {
    //shootingMotor.set(shootingMotor.get());
    //loadingMotor.set(loadingMotor.get());
  }

  public double normilizeSpeed(double speed){
    speed = Math.min(speed, 1);
    speed = Math.max(speed,-1);
    return speed;
  }

  public double getLoadingDistance() {
      return loadingDistance;
  }

  public double getHoldingDistance() {
      return holdingDistance;
  }

  //------------------Shooter Functions------------------


  public void load(){
    load(loadSpeed);
  }
  
  public void load(double speed){
    loadingMotor.set(speed);
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
    shootingMotor.set(State.Idle.getSpeed());
    this.currentState = State.Idle;
  } 
  
  //------------------Getter Functions------------------

  public State getState(){
    return currentState;
  }

}
