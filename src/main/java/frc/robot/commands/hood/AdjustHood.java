// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.core751.CoreConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hood.BangBang;


public class AdjustHood extends CommandBase {
  private final Hood hood;
  private Joystick joystick = CoreConstants.driverStick;
  private int axis;
  


  public AdjustHood(Hood subsystem, int axis) {
    this.hood = subsystem;
    this.axis = axis;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double value = -joystick.getRawAxis(axis);
    if(Math.abs(value)<= 0.1){
      value = 0;
    }

    if(hood.getAngle() > 35){
      value = Math.min(0,value);
    }
    

    if(joystick.getRawButton(1)){
      hood.setAngle(30);
    }
    else if(joystick.getRawButton(4)){
      hood.setAngle(5);
    }else{
      hood.adjustAngle(value);
      if(value != 0){
        hood.setState(BangBang.Idle);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
