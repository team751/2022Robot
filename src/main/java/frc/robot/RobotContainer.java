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
import frc.robot.commands.ExampleCommand;
import frc.robot.core751.CoreConstants;
import frc.robot.core751.driveTrainConstants;
import frc.robot.core751.commands.JoystickRecorder;
import frc.robot.core751.commands.RunMotorTest;
import frc.robot.core751.commands.camera.LimeLight.SwitchCameraMode;
import frc.robot.core751.commands.drivetrain.ReversableArcadeDrive;
import frc.robot.core751.subsystems.DifferentialDriveTrain;
import frc.robot.core751.subsystems.DriveSubsystem;
import frc.robot.core751.subsystems.LimeLight;
import frc.robot.core751.subsystems.MotorTest;
import frc.robot.core751.subsystems.DifferentialDriveTrain.SmartControllerProfile;
import frc.robot.core751.subsystems.MotorTest.motType;
import frc.robot.subsystems.ExampleSubsystem;
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

  DriveSubsystem m_robotDrive = new DriveSubsystem();

  //private final DifferentialDriveTrain differentialDriveTrain = new DifferentialDriveTrain(CoreConstants.leftDrivetrainIDs, CoreConstants.rightDrivetrainIDs, CoreConstants.driveTrainMotorType, CoreConstants.driveMotorProfile, CoreConstants.driveInvertLeft, CoreConstants.driveInvertRight);
  //private final ReversableArcadeDrive reversableArcadeDrive = new ReversableArcadeDrive(CoreConstants.driverStick, differentialDriveTrain);
  //private final MotorTest motorTest = new MotorTest(motType.Falcon500,0);
  //private final RunMotorTest rMotorTest = new RunMotorTest(motorTest);
  //private pdpTest pTest = new pdpTest(new int[] {0,15});
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
     //differentialDriveTrain.setDefaultCommand(reversableArcadeDrive);
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
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                driveTrainConstants.ksVolts,
                driveTrainConstants.kvVoltSecondsPerMeter,
                driveTrainConstants.kaVoltSecondsSquaredPerMeter),
            driveTrainConstants.kDriveKinematics,
            2);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                driveTrainConstants.kMaxSpeedMetersPerSecond,
                driveTrainConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(driveTrainConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            new RamseteController(driveTrainConstants.kRamseteB, driveTrainConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                driveTrainConstants.ksVolts,
                driveTrainConstants.kvVoltSecondsPerMeter,
                driveTrainConstants.kaVoltSecondsSquaredPerMeter),
            driveTrainConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(driveTrainConstants.kPDriveVel, 0, 0),
            new PIDController(driveTrainConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }
  public Command getTeleopCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
