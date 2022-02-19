package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.core751.subsystems.LimeLight;
import frc.robot.core751.subsystems.PhotonVision;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends CommandBase{
    
    Shooter shooter;
    Hood hood;
    PhotonVision photonVision;

    public AutoShoot(Shooter shooter, Hood hood, PhotonVision photonVision){
        this.shooter = shooter;
        this.hood = hood;
        this.photonVision = photonVision;
    }



    public void shoot(){
        
    }


    private double distanceToSpeed(double inches){
        return inches; //Replace this with fitted curve
    }

    private double distanceToAngle(double inches){
        return inches; //Replace this with fitted curve
    }
}
