package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.core751.wrappers.AnalogDistanceSensor;
import frc.robot.core751.wrappers.wTalonFX;

public class Shooter extends SubsystemBase {
    
    private wTalonFX shootMotor;
    private CANSparkMax loadMotor;

    private AnalogDistanceSensor topSensor;
    private AnalogDistanceSensor middleSensor;
    private AnalogDistanceSensor bottomSensor;

    private double loadSpeed = Constants.loadSpeed;

    public enum FlyWheelState{
        idle(Constants.shooterIdleSpeed),
        spinUp(Constants.spinUpSpeed),
        off(0);

        private double speed;
        private FlyWheelState(double speed){
          this.speed = speed;
        }
        public void setSpeed(double speed){
          this.speed = speed;
        }
        public double getSpeed(){
          return this.speed;
        }

    }

    public Shooter(int shootMotorId,int loadMotorId, AnalogDistanceSensor topSensor, AnalogDistanceSensor middleSensor, AnalogDistanceSensor bottomSensor){
        this.shootMotor = new wTalonFX(shootMotorId);
        this.loadMotor = new CANSparkMax(loadMotorId, MotorType.kBrushless);

        this.topSensor = topSensor;
        this.middleSensor = middleSensor;
        this.bottomSensor = bottomSensor;
    }

    public void load(){
        load(this.loadSpeed);
    }

    public void load(double loadSpeed){
        loadMotor.set(loadSpeed);
    }

    public void spinUp(){

    }

    public void spinUp(double spinUpSpeed){

    }

    public void idle(){

    }

    public void idle(double idleSpeed){
        
    }

    //Getter Commands

    public double getTopSensor() {
        return topSensor.getDistance();
    }

    public double getMiddleSensor() {
        return middleSensor.getDistance();
    }
    public double getBottomSensor() {
        return bottomSensor.getDistance();
    }




}
