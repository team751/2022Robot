package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.core751.CoreConstants;
import frc.robot.core751.wrappers.OverrideableJoystick;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase{
    
    Intake intake;
    Joystick joystick = CoreConstants.driverStick;

    int intakeMotorButton;
    int armButtonUp;
    int armButtonDown;

    double intakeSpeed = Constants.intakeSpeed;
    
    public IntakeCommand(Intake intake,int intakeMotorButton, int armButtonUp, int armButtonDown){
        this.intake = intake;

        this.intakeMotorButton = intakeMotorButton;
        this.armButtonDown = armButtonDown;
        this.armButtonUp = armButtonUp;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if(joystick.getRawButton(intakeMotorButton)){
            intake.intake(intakeSpeed);
        }else{
            intake.intake(Constants.intakeIdleSpeed);
        }

        // if(joystick.getRawButton(armButtonDown)){
        //     intake.moveArmDown();
        // }else if (joystick.getRawButton(armButtonUp)){
        //     intake.moveArmUp();
        // }
    }

}
