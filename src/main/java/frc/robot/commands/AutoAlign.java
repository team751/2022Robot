package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.core751.auton.CameraAdjustToAngle;
import frc.robot.core751.auton.*;
import frc.robot.core751.subsystems.camera.PhotonVision;
import frc.robot.core751.subsystems.drivetrain.TrajectoryDrive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterDeprecated;

public class AutoAlign {

    private TrajectoryDrive trajectoryDrive;
    private PhotonVision photonVision;
    private Shooter shooter; 
    private Hood hood;

    public AutoAlign(TrajectoryDrive trajectoryDrive, PhotonVision photonVision, Shooter shooter, Hood hood){
        this.trajectoryDrive = trajectoryDrive;
        this.photonVision = photonVision;
        this.shooter = shooter;
        this.hood = hood;
    }


    public CommandBase getCommand(){
        double angleToTarget = Math.atan(trajectoryDrive.getPose().getX() / trajectoryDrive.getPose().getY());
        AdjustToAngle adjustToAngle = new AdjustToAngle(trajectoryDrive, angleToTarget, 0.5, 25,10); //Change max accel
        MillisecondWaitCommand waitCommand = new MillisecondWaitCommand(100);
        CameraAdjustToAngle cameraAngleAdjust = new CameraAdjustToAngle(trajectoryDrive, photonVision, 0.5, 25, 0.01);
        //AutoShoot autoShoot = new AutoShoot(shooter, hood, photonVision);
        SequentialCommandGroup commands = new SequentialCommandGroup(adjustToAngle,waitCommand,cameraAngleAdjust);
        return commands;
    }
}
