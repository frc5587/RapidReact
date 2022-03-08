package frc.robot.subsystems;

import frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
Use a motor to control a wheel that will move balls into & out of the kicker
*/
public class Intake extends SubsystemBase {
    private CANSparkMax motor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();
    
    private double setpoint = 0;

    public Intake() {
        configureIntakeSpark();
    }

    public void configureIntakeSpark() {
        resetEncoders();
        motor.restoreFactoryDefaults();
        motor.setInverted(IntakeConstants.MOTOR_INVERTED);
        motor.setIdleMode(IdleMode.kBrake);
    }

    public void setVelocity(double velocity) {
        setpoint = velocity;
    }

    public void stop() {
        setVelocity(0);
    }
    
    public void resetEncoders() {
        encoder.setPosition(0);
    }

    /*
    Convert velocity RPM to meters per second of the surface of the wheel
    */
    protected double getMeasurement() {
        return (encoder.getVelocity() / 60) * (2 * Math.PI) * (IntakeConstants.WHEEL_RADIUS / IntakeConstants.GEARING);
    }

    @Override
    public void periodic() {
        motor.setVoltage(IntakeConstants.INTAKE_FF.calculate(setpoint) - IntakeConstants.PID.calculate(setpoint - getMeasurement()));
    }
}
