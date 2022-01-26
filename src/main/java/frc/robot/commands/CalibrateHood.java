// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Hood;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class CalibrateHood extends CommandBase {
  private final Hood hood;

  
  public CalibrateHood(Hood subsystem) {
    hood = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hood.setSafeMode(false);
    hood.adjustAngle(0.5);
    try {
      wait(10000);
    } catch (InterruptedException e) {  e.printStackTrace(); }
    hood.setSafeMode(true);
    System.out.println("!!Calibration Factor: " + hood.getCalibrationFactor());
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
