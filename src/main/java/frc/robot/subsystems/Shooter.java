package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.core751.CoreConstants;
import frc.robot.core751.wrappers.AnalogDistanceSensor;
import frc.robot.core751.wrappers.wTalonFX;

public class Shooter extends SubsystemBase {
    
    private wTalonFX shootMotor;
    private CANSparkMax loadMotor;

    private AnalogDistanceSensor topSensor;
    private AnalogDistanceSensor bottomSensor;

    private double setpoint = 0;

    private double loadSpeed = Constants.loadSpeed;

    public Shooter(int shootMotorId,int loadMotorId, int topSensorId, int bottomSensorId){
        this.shootMotor = new wTalonFX(shootMotorId);
        this.loadMotor = new CANSparkMax(loadMotorId, MotorType.kBrushless);

        this.topSensor = new AnalogDistanceSensor(topSensorId);
        this.bottomSensor = new AnalogDistanceSensor(bottomSensorId);
    }

    public void load(){
        load(this.loadSpeed);
    }

    public void load(double loadSpeed){
        loadMotor.set(loadSpeed);
    }


    //Using https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-flywheel-walkthrough.html

    // Volts per (rotations per second)
    private static final double kFlywheelKv = 0.10922; //Normal: 0.10922 //Flywheel 0.10956

    // Volts per (rotations per second squared)
    private static final double kFlywheelKa = 0.0046519; //Normal: 0.0046519 //Flywheel: 0.0082591

    private final LinearSystem<N1, N1, N1> m_flywheelPlant =
        LinearSystemId.identifyVelocitySystem(kFlywheelKv, kFlywheelKa);

    private final KalmanFilter<N1, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          m_flywheelPlant,
          VecBuilder.fill(3.0), // How accurate we think our model is
          VecBuilder.fill(0.01), // How accurate we think our encoder data is
          0.020);

    // A LQR uses feedback to create voltage commands.
    private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
        new LinearQuadraticRegulator<>(
            m_flywheelPlant,
            VecBuilder.fill(8), // qelms. Velocity error tolerance, in radians per second. Decrease
            // this to more heavily penalize state excursion, or make the controller behave more
            // aggressively.
            VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12 is a good
            // starting point because that is the (approximate) maximum voltage of a battery.
            0.020); 

    private final LinearSystemLoop<N1, N1, N1> m_loop =
        new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12.0, 0.020);

    public void setFlywheelSpeed(double rotationsPerSecond){
        setpoint = rotationsPerSecond;
        SmartDashboard.putNumber("Talon Setpoint", rotationsPerSecond);
        SmartDashboard.putNumber("Talon Actual", shootMotor.getVelocity());

        m_loop.setNextR(VecBuilder.fill(rotationsPerSecond));
        // Correct our Kalman filter's state vector estimate with encoder data.
        m_loop.correct(VecBuilder.fill(shootMotor.getVelocity()));
        // Update our LQR to generate new voltage commands and use the voltages to predict the next
        // state with out Kalman filter.
        m_loop.predict(0.020);

        double nextVoltage = m_loop.getU(0);
        shootMotor.setVoltage(nextVoltage);
    }

    //----------------Getter Commands--------------------

    public double getTopSensor() {
        return topSensor.getDistance();
    }

    public double getBottomSensor() {
        return bottomSensor.getDistance();
    }

    public boolean getTopSensorHasTarget(){
        return topSensor.hasTraget();
    }

    public boolean getBottomSensorHasTarget(){
        return bottomSensor.hasTraget();
    }

    public boolean atSetpoint(){
        return (Math.abs(shootMotor.getVelocity() - setpoint) < 3); //Tolerance 
    }




}
