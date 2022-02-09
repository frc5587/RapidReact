package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import frc.robot.Constants;

public class Shooter extends ProfiledPIDSubsystem {
    private final TalonFX motor = new TalonFX(Constants.ShooterConstants.SHOOTER_MOTOR);

    public Shooter() {
        super(new ProfiledPIDController(
            Constants.ShooterConstants.PID.kP,
            Constants.ShooterConstants.PID.kI,
            Constants.ShooterConstants.PID.kD,
            Constants.ShooterConstants.CONSTRAINTS
        ));

        motor.setInverted(true); // TODO Check if this needs to be inverted
    }

    public void flyWheelForwards() {
        motor.set(ControlMode.PercentOutput, Constants.ShooterConstants.FORWARDS_VELOCITY);
    }

    public void flyWheelBackwards() {
        motor.set(ControlMode.PercentOutput, Constants.ShooterConstants.BACKWARDS_VELOCITY);
    }

    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    public void resetEncoders() {
        motor.setSelectedSensorPosition(0);
    }

    protected double getPositionDegrees() {
        return (motor.getSelectedSensorPosition() / Constants.ShooterConstants.GEARING / Constants.ShooterConstants.ENCODER_CPR);
    }

    protected double getPositionRadians() {
        return Math.toRadians(getPositionDegrees());
    }

    @Override
    protected double getMeasurement() {
        return getPositionRadians();
    }

    // TODO Output argument should actually be used for PID
    @Override
    protected void useOutput(double output, State setpoint) {
        flyWheelForwards();
    }
}
