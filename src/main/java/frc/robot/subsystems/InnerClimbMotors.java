package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants.ClimbConstants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.frc5587.lib.subsystems.FPIDSubsystem;

import com.revrobotics.RelativeEncoder;

public class ClimbMotor extends FPIDSubsystem {
    private static CANSparkMax motor = new CANSparkMax(ClimbConstants.INNER_CLIMB_RIGHT_MOTOR,
            MotorType.kBrushless);

    private static RelativeEncoder encoder = motor.getEncoder();

    public ClimbMotor() {
        super(new ProfiledPIDController(
                Constants.ClimbConstants.PID.kP,
                Constants.ClimbConstants.PID.kI,
                Constants.ClimbConstants.PID.kD,
                Constants.ClimbConstants.CONSTRAINTS));
        configureClimbSpark();
    }

    public void configureClimbSpark() {
        innerClimbRightMotor.restoreFactoryDefaults();
        innerClimbLeftMotor.restoreFactoryDefaults();

        innerClimbRightMotor.setInverted(ClimbConstants.MOTOR_INVERTED);
        innerClimbLeftMotor.setInverted(ClimbConstants.MOTOR_INVERTED);

        innerClimbRightMotor.setIdleMode(IdleMode.kBrake);
        innerClimbLeftMotor.setIdleMode(IdleMode.kBrake);
    }

    // uses radians per second
    public void setSpeed(double innerClimbSpeed) {
        innerClimbRightMotor.set(innerClimbSpeed);
        innerClimbLeftMotor.set(innerClimbSpeed);
    }

    public void setThrottle(double throttle) {
        innerClimbRightMotor.set(throttle);
        innerClimbLeftMotor.set(throttle);
    }

    public void stopClimb() {
        innerClimbRightMotor.set(0);
        innerClimbLeftMotor.set(0);
    }

    public void resetEncoders() {
        innerClimbRightEncoder.setPosition(0);
        innerClimbLeftEncoder.setPosition(0);
    }

    protected double getPositionDegrees() {
        return Math.toRadians(getPositionDegrees());
    }

    // TODO - I don't know what this means
    protected double getPositionRadians() {
        return (innerClimbRightEncoder.getPosition() / Constants.ClimbConstants.GEARING
                / Constants.ClimbConstants.ENCODER_CPR);
    }

    @Override
    public double getMeasurement() {
        return getPositionRadians();
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        setThrottle(output);
    }

    @Override
    public void configureMotors() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getEncoderPosition() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getEncoderVelocity() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void setEncoderPosition(double position) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double rotationsToMeasurement(double rotations) {
        // TODO Auto-generated method stub
        return 0;
    }
}