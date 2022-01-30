// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.core751.wrappers.OverrideableJoystick;
import frc.robot.subsystems.Hood;


public class AdjustHood extends CommandBase {
  private final Hood hood;
  private OverrideableJoystick joystick;
  
  private int joystickPort;
  


  public AdjustHood(Hood subsystem,OverrideableJoystick joystick, int joystickPort) {
    this.hood = subsystem;
    this.joystick = joystick;
    this.joystickPort = joystickPort;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hood.adjustAngle(joystick.getRawAxis(joystickPort));
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
