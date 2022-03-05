package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    
    WPI_VictorSPX windowMotor;
    CANSparkMax intakeMotor;

    private DigitalInput topLimitSwitch;
    private DigitalInput bottomLimitSwitch;


    public Intake(int intakeMotor, int windowMotorId){
        this.intakeMotor = new CANSparkMax(intakeMotor, MotorType.kBrushless);
       
        this.windowMotor = new WPI_VictorSPX(windowMotorId);
    }

    @Override
    public void periodic() {
            // if(this.topLimitSwitch.get() && windowMotor.get() > 0){
            //     windowMotor.set(0);
            // }
            // if(this.bottomLimitSwitch.get() && windowMotor.get() < 0){
            //     windowMotor.set(0);
            // }
        }


    public void moveArm(double speed){
        //if(!this.topLimitSwitch.get()){
        windowMotor.set(speed);
        //}
    }

    public void intake(double speed){
        intakeMotor.set(speed);
    }



}
