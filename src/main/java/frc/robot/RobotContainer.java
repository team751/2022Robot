// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
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
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootingCommandDepracated;
import frc.robot.commands.hood.AdjustHood;
import frc.robot.core751.CoreConstants;
import frc.robot.core751.auton.AdjustToAngle;
import frc.robot.core751.auton.AutonDrive;
import frc.robot.core751.commands.JoystickRecorder;
import frc.robot.core751.commands.RunMotorTest;
import frc.robot.core751.commands.camera.LimeLight.SwitchCameraMode;
import frc.robot.core751.commands.drivetrain.PIDdrive;
import frc.robot.core751.commands.drivetrain.ReversableArcadeDrive;
import frc.robot.core751.subsystems.MotorTest;
import frc.robot.core751.subsystems.MotorTest.motType;
import frc.robot.core751.subsystems.camera.LimeLight;
import frc.robot.core751.subsystems.drivetrain.DifferentialDriveTrain;
import frc.robot.core751.subsystems.drivetrain.TrajectoryDrive;
import frc.robot.core751.wrappers.wTalonFX;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
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

  private final TrajectoryDrive differentialDriveTrain = new TrajectoryDrive(CoreConstants.leftDrivetrainIDs, CoreConstants.rightDrivetrainIDs, CoreConstants.driveInvertLeft, CoreConstants.driveInvertRight);
  private final ReversableArcadeDrive reversableArcadeDrive = new ReversableArcadeDrive(CoreConstants.driverStick, differentialDriveTrain);
  //private final PIDdrive pidDrive = new PIDdrive(differentialDriveTrain, 1);

  private final Hood hood = new Hood(9, 2, 3, 2);
  private final AdjustHood adjustHood = new AdjustHood(hood, 3);


  // private final MotorTest motorTest = new MotorTest(motType.Falcon500,10);
  // private final RunMotorTest rMotorTest = new RunMotorTest(motorTest);

  private final Shooter shooter = new Shooter(10,7,1,0); //Bottom Sensor:0 Top Sensor:1
  private final ShootingCommandDepracated shootingCommand = new ShootingCommandDepracated(shooter, CoreConstants.Controller.A.getButtonMapping(), CoreConstants.Controller.B.getButtonMapping(), 0, 0);

  //private final Intake intake = new Intake(8, 9);
  //private final IntakeCommand intakeCommand = new IntakeCommand(intake, CoreConstants.Controller.Y.getButtonMapping() , CoreConstants.Controller.RB.getButtonMapping(), CoreConstants.Controller.LB.getButtonMapping());

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

     differentialDriveTrain.setDefaultCommand(reversableArcadeDrive);
     //differentialDriveTrain.setDefaultCommand(pidDrive);
     shooter.setDefaultCommand(shootingCommand);
     hood.setDefaultCommand(adjustHood);
     //intake.setDefaultCommand(intakeCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    //return new AdjustToAngle(differentialDriveTrain, 90, 1, 1, 1);

    return null;
    
  }
  public Command getTeleopCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
