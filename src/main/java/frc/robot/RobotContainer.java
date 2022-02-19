// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootingCommandDepracated;
import frc.robot.core751.CoreConstants;
import frc.robot.core751.commands.JoystickRecorder;
import frc.robot.core751.commands.RunMotorTest;
import frc.robot.core751.commands.camera.LimeLight.SwitchCameraMode;
import frc.robot.core751.commands.drivetrain.ReversableArcadeDrive;
import frc.robot.core751.subsystems.DifferentialDriveTrain;
import frc.robot.core751.subsystems.LimeLight;
import frc.robot.core751.subsystems.MotorTest;
import frc.robot.core751.subsystems.DifferentialDriveTrain.SmartControllerProfile;
import frc.robot.core751.subsystems.MotorTest.motType;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterDeprecated;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

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

  
  private final ShooterDeprecated shooter = new ShooterDeprecated(7,0);
  private final ShootingCommandDepracated shootingCommand = new ShootingCommandDepracated(shooter, CoreConstants.driverStick, CoreConstants.Controller.A.getButtonMapping(), CoreConstants.Controller.B.getButtonMapping(), 0, 0);

  //private final Intake intake = new Intake(8, 9);
  //private final IntakeCommand intakeCommand = new IntakeCommand(intake, CoreConstants.Controller.Y.getButtonMapping() , CoreConstants.Controller.LB.getButtonMapping(), CoreConstants.Controller.RB.getButtonMapping());
  //private pdpTest pTest = new pdpTest(new int[] {0,15})
  //private final LimeLight limeLight = new LimeLight();
  //private final SwitchCameraMode switchCameraMode = new SwitchCameraMode(limeLight, CoreConstants.driverStick, 1);

  

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

     //motorTest.setDefaultCommand(rMotorTest);
     differentialDriveTrain.setDefaultCommand(reversableArcadeDrive);
     shooter.setDefaultCommand(shootingCommand);
     //intake.setDefaultCommand(intakeCommand);
     //SmartDashboard.putData(pdp);
     //limeLight.setDefaultCommand(switchCameraMode);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    return null;
  }
  public Command getTeleopCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
