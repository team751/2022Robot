// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.core751.CoreConstants;
import frc.robot.core751.commands.JoystickRecorder;
import frc.robot.core751.commands.RunMotorTest;
import frc.robot.core751.commands.drivetrain.ReversableArcadeDrive;
import frc.robot.core751.subsystems.DifferentialDriveTrain;
import frc.robot.core751.subsystems.MotorTest;
import frc.robot.core751.subsystems.DifferentialDriveTrain.SmartControllerProfile;
import frc.robot.core751.subsystems.MotorTest.motType;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //public final JoystickRecorder joystickRecorder = new JoystickRecorder(CoreConstants.driverStick);

  private final DifferentialDriveTrain differentialDriveTrain = new DifferentialDriveTrain(CoreConstants.leftDrivetrainIDs, CoreConstants.rightDrivetrainIDs, CoreConstants.driveTrainMotorType, CoreConstants.driveMotorProfile, CoreConstants.driveInvertLeft, CoreConstants.driveInvertRight);
  private final ReversableArcadeDrive reversableArcadeDrive = new ReversableArcadeDrive(CoreConstants.driverStick, differentialDriveTrain);
  //private final MotorTest motorTest = new MotorTest(motType.Falcon500,0);
  //private final RunMotorTest rMotorTest = new RunMotorTest(motorTest);


  private final PowerDistribution pdp = new PowerDistribution();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

     differentialDriveTrain.setDefaultCommand(reversableArcadeDrive);
     //SmartDashboard.putData(pdp);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
