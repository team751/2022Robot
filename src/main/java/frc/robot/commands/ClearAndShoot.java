package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.core751.CoreConstants;
import frc.robot.core751.auton.MillisecondWaitCommand;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class ClearAndShoot extends CommandBase {
   
    public Shooter shooter;
    public Hood hood;

    public enum State{
        clear,
        wait1,
        shoot1,
        wait2,
        shoot2,
        finished;
    }

    MillisecondWaitCommand wait1 = new MillisecondWaitCommand(50);
    MillisecondWaitCommand wait2 = new MillisecondWaitCommand(500);

    AutoShoot shoot1;
    AutoShoot shoot2;

    public State currentState = State.clear;

    /**
     * 
     * @param shooter
     * @param wheelSpeed Rotations per Second
     */
    public ClearAndShoot(Shooter shooter, Hood hood){
        this.shooter = shooter;
        this.hood = hood; 
        shoot1 = new AutoShoot(shooter, hood, CoreConstants.photonVision);
        shoot2 = new AutoShoot(shooter, hood, CoreConstants.photonVision);
    } 

    @Override
    public void execute() {
        switch(currentState){
            case clear:
                shooter.setFlywheelSpeed(-10);
                shooter.load(-0.25);
                wait1.schedule();
                currentState = State.wait1;
                break;
            case wait1:
                if(wait1.isFinished()){
                    shooter.setFlywheelSpeed(0);
                    shooter.load(0);
                    shoot1.schedule();
                    currentState = State.shoot1;
                    
                }
                break;
            case shoot1: //change to use sensors for knowing when shot instead of timer. 
                if(shoot1.isFinished()){
                    wait2.schedule();
                    currentState = State.wait2;
                }
                break;
            case wait2: //Change to seeing when the speed is right.
                if(wait2.isFinished()){
                    shoot1.schedule();
                    currentState = State.shoot2;
                }
                break;
            case shoot2: //change to use sensors for knowing when shot instead of timer. 
                if(shoot1.isFinished()){
                    wait2.schedule();
                    currentState = State.finished;
                }
                break;
        }
    }


    @Override
    public boolean isFinished() {
        return currentState == State.finished;
    }
}
