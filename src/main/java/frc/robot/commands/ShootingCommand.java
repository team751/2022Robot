// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.core751.CoreConstants;
import frc.robot.core751.wrappers.OverrideableJoystick;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.State;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** An example command that uses an example subsystem. */
public class ShootingCommand extends CommandBase {

  private final Shooter subsytem;
  private Joystick joystick;

  private int shootButton;
  private int spinUpButton;
  private int idleButton;
  private int loadButton;


  public ShootingCommand(Shooter subsystem,Joystick joystick,int shootButton,int loadButton,int spinUpButton, int idleButton) {
    this.subsytem = subsystem;
    this.joystick = joystick;
    this.shootButton = shootButton;
    this.loadButton = loadButton;

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

      State.SpinUp.setSpeed(SmartDashboard.getNumber("SpinUpSpeed",Constants.intakeSpeed));

      if(joystick.getRawButton(shootButton)){
        //subsytem.load();
        subsytem.spinUp(0.75);
      }else{
        subsytem.idle();
      }
      if(joystick.getRawButton(loadButton)){
        subsytem.load();
      } else {
        subsytem.load(0);
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
