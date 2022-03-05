package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.core751.subsystems.camera.LimeLight;
import frc.robot.core751.subsystems.camera.PhotonVision;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterDeprecated;

public class AutoShoot extends CommandBase{
    
    Shooter shooter;
    Hood hood;
    PhotonVision photonVision;

    double distance;

    private enum shotStatus{
        NoBall,
        LoadedBall,
        Shot
    }

    private shotStatus status = shotStatus.NoBall;

    public AutoShoot(Shooter shooter, Hood hood, PhotonVision photonVision){
        this.shooter = shooter;
        this.hood = hood;
        this.photonVision = photonVision;

        distance = photonVision.getDistance(); //Maybe make this fluctionation protected??
    }

    @Override
    public void execute() {
        shooter.setFlywheelSpeed(distanceToSpeed(distance));
        hood.setAngle(distanceToAngle(distance));
        if(hood.atSetpoint() && shooter.atSetpoint()){
            shooter.load(0.25);
        }
        
        if(shooter.getTopSensorHasTarget()){
            status = shotStatus.LoadedBall;
        }else if(shooter.getTopSensorHasTarget() && status == shotStatus.LoadedBall){
            status = shotStatus.Shot;
        }
    }

    @Override
    public boolean isFinished() {
        return status == shotStatus.Shot;
    }


    private double distanceToSpeed(double meters){ 
        return 20; //Replace this with fitted curve
    }

    private double distanceToAngle(double meters){
        return 20; //Replace this with fitted curve
    }
}
