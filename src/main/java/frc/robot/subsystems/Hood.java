// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.core751.wrappers.wAnalogPotentiometer;

public class Hood extends SubsystemBase {

  public enum BangBang{
    Idle,
    Updating
  }

  private VictorSPX hoodMotor;
  public double hoodAngle = 0;
  
  private DigitalInput bottomLimitSwitch;
  private wAnalogPotentiometer pot;

  private double softAngleMax = Constants.softAngleMax;
  
  private BangBang state = BangBang.Idle;

  private PIDController pidController = new PIDController(Constants.angleAdjustSpeed / 10, 0, 0); //At 10 deg error run at adjust speed.

  private double tolerance = Constants.angleTolerance;

  public Hood(int hoodMotorId, int bottomSwitchPin,int potPin) {
    pidController.setTolerance(tolerance);
    hoodMotor = new VictorSPX(hoodMotorId);
    hoodMotor.setInverted(true);
    //this.topLimitSwitch = new DigitalInput(topSwitchPin);
    this.bottomLimitSwitch = new DigitalInput(bottomSwitchPin);
    this.pot = new wAnalogPotentiometer(potPin, -10,+1070);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Pot Angle", pot.getAngle());
    //SmartDashboard.putNumber("5v", RobotController.getVoltage5V());
  
    hoodAngle = pot.getAngle();

    //Making sure we don't over extend
    //Checking for contact every "frame" is there a better way?

    // if(this.bottomLimitSwitch.get()){
    //   adjustAngle(0);
    //   state = BangBang.Idle;
    // } 

    //For PID cycle.
    if(state == BangBang.Updating){
      if (!pidController.atSetpoint()){
        adjustAngle(pidController.calculate(hoodAngle));
      } else {
        adjustAngle(0);
        System.out.println("Hood IDLE");
        state = BangBang.Idle;
      }
    }
  }
  
  public void setState(BangBang state) {
      this.state = state;
  }

  //------------------Hood Functions------------------
  public void setAngle(double angle){ 
    angle = Math.min(angle, softAngleMax);
    angle = Math.max(angle, 0);

    pidController.setSetpoint(angle);
    this.state = BangBang.Updating;

  }

  public void adjustAngle(double speed){
    // if(this.bottomLimitSwitch.get()){
    //   speed = Math.min(0, speed);
    // }
    hoodMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }

//------------------Getter Functions------------------
  public double getAngle(){
    return hoodAngle;
  }

  public BangBang getState(){
    return state;
  }

  public boolean atSetpoint(){
    return pidController.atSetpoint();
  }


}
