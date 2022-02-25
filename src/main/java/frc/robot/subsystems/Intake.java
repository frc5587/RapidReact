package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;

/**
Use a motor to control a wheel in order to intake & outtake balls to the robot.
*/
public class Intake extends ProfiledPIDSubsystem {
    private static CANSparkMax motor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);
    private static RelativeEncoder encoder = motor.getEncoder();

    public Intake() {
        super(new ProfiledPIDController(
            IntakeConstants.PID.kP,
            IntakeConstants.PID.kI,
            IntakeConstants.PID.kD,
            IntakeConstants.CONSTRAINTS
        ));
        configureMotors();
    }

    public void setVelocity(double velocity) {
        setGoal(new TrapezoidProfile.State(0, velocity));
    }

    public void configureMotors() {
        resetEncoders();
        motor.restoreFactoryDefaults();
        motor.setInverted(IntakeConstants.INVERTED);
        motor.setIdleMode(IdleMode.kBrake);
    }

    public void moveWithThrottle(double throttle) {
        motor.set(throttle);
    }

    public void stop() {
        motor.set(0);
    }
    
    public void resetEncoders() {
        encoder.setPosition(0);
    }

    /*
    Convert velocity RPM to meters per second
    */
    @Override
    protected double getMeasurement() {
        return encoder.getVelocity() / 60 * 2 * Math.PI * Units.inchesToMeters(IntakeConstants.WHEEL_RADII) / IntakeConstants.GEARING;
    }

    /*
    Use PID output and computed Feedforward to control voltage output to the motor
    */
    @Override
    protected void useOutput(double output, State setpoint) {     
        motor.setVoltage(IntakeConstants.INTAKE_FF.calculate(setpoint.velocity) - output);
    }
}
