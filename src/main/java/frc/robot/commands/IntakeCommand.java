package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        SmartDashboard.putNumber("Speed Intake", Constants.intakeSpeed);

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if(joystick.getRawButton(intakeMotorButton)){
            intake.intake(SmartDashboard.getNumber("Speed Intake", Constants.intakeSpeed));
        }else{
            intake.intake(Constants.intakeIdleSpeed);
        }

        if(joystick.getRawButton(armButtonDown)){
            intake.moveArm(1);
        }else if (joystick.getRawButton(armButtonUp)){
            intake.moveArm(-1);
        }else{
            intake.moveArm(0);
        }
    }

}
