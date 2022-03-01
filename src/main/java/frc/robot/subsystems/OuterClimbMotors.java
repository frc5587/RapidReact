package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants.ClimbConstants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class OuterClimbMotors extends ProfiledPIDSubsystem {
    private static CANSparkMax outerClimbRightMotor = new CANSparkMax(ClimbConstants.OUTER_CLIMB_RIGHT_MOTOR,
            MotorType.kBrushless);
    private static CANSparkMax outerClimbLeftMotor = new CANSparkMax(ClimbConstants.OUTER_CLIMB_LEFT_MOTOR,
            MotorType.kBrushless);

    private static RelativeEncoder outerClimbRightEncoder = outerClimbRightMotor.getEncoder();
    private static RelativeEncoder outerClimbLeftEncoder = outerClimbLeftMotor.getEncoder();

    public OuterClimbMotors() {
        super(new ProfiledPIDController(
                Constants.ClimbConstants.PID.kP,
                Constants.ClimbConstants.PID.kI,
                Constants.ClimbConstants.PID.kD,
                Constants.ClimbConstants.CONSTRAINTS));
        configureClimbSpark();
    }

    public void configureClimbSpark() {
        outerClimbRightMotor.restoreFactoryDefaults();
        outerClimbLeftMotor.restoreFactoryDefaults();

        outerClimbRightMotor.setInverted(ClimbConstants.MOTOR_INVERTED);
        outerClimbLeftMotor.setInverted(ClimbConstants.MOTOR_INVERTED);

        outerClimbRightMotor.setIdleMode(IdleMode.kBrake);
        outerClimbLeftMotor.setIdleMode(IdleMode.kBrake);
    }

    // uses radians per second
    public void setSpeed(double outerClimbSpeed) {
        outerClimbRightMotor.set(outerClimbSpeed);
        outerClimbLeftMotor.set(outerClimbSpeed);
    }

    public void setThrottle(double throttle) {
        outerClimbRightMotor.set(throttle);
        outerClimbLeftMotor.set(throttle);
    }

    public void stopClimb() {
        outerClimbRightMotor.set(0);
        outerClimbLeftMotor.set(0);
    }

    public void resetEncoders() {
        outerClimbRightEncoder.setPosition(0);
        outerClimbLeftEncoder.setPosition(0);
    }

    protected double getPositionDegrees() {
        return Math.toRadians(getPositionDegrees());
    }

    // TODO - I don't know what this means
    protected double getPositionRadians() {
        return (outerClimbRightEncoder.getPosition() / Constants.ClimbConstants.GEARING
                / Constants.ClimbConstants.ENCODER_CPR);
    }

    @Override
    protected double getMeasurement() {
        return getPositionRadians();
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        System.out.println("Come back to this pls");
    }
}