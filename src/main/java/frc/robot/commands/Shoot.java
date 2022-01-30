// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.core751.wrappers.OverrideableJoystick;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.State;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Shoot extends CommandBase {

  private final Shooter subsytem;
  private OverrideableJoystick joystick;

  private int shootButton;
  private int spinUpButton;
  private int idleButton;


  public Shoot(Shooter subsystem,OverrideableJoystick joystick,int shootButton,int spinUpButton, int idleButton) {
    this.subsytem = subsystem;
    this.joystick = joystick;
    this.shootButton = shootButton;

    this.spinUpButton = spinUpButton;
    this.idleButton = idleButton; 

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      State.SpinUp.setSpeed(SmartDashboard.getNumber("SpinUpSpeed",1.0));
      if(joystick.getRawButton(shootButton)){

        if(subsytem.getState() != State.SpinUp){
          subsytem.spinUp();
          try {
            wait(500);
          } catch (InterruptedException e) {}
        }
        subsytem.load();

      }

      if(joystick.getPOV(idleButton) != -1){
        subsytem.idle();
      }

      if(joystick.getPOV(spinUpButton) != -1){
        subsytem.spinUp();
      }

  }

  //-----------------Unused for now-----------------

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
