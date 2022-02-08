package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        if(this.topLimitSwitch.get() || this.bottomLimitSwitch.get()){
            windowMotor.set(0);
            }
    }

    public void moveArmUp(){
        if(!this.topLimitSwitch.get()){
        windowMotor.set(1);
        }
    }

    public void moveArmDown(){
        if(!this.bottomLimitSwitch.get()){
            windowMotor.set(-1);
            }
    }

    public void intake(double speed){
        intakeMotor.set(speed);
    }



}
