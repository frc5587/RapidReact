package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.KickerConstants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class Kicker extends ProfiledPIDSubsystem {
    private static CANSparkMax rightKickerMotor = new CANSparkMax(KickerConstants.RIGHT_KICKER_MOTOR, MotorType.kBrushless);
    private static CANSparkMax leftKickerMotor = new CANSparkMax(KickerConstants.LEFT_KICKER_MOTOR, MotorType.kBrushless);

    private static RelativeEncoder rightKickerEncoder = rightKickerMotor.getEncoder();
    private static RelativeEncoder leftKickerEncoder = leftKickerMotor.getEncoder();
    
    public Kicker() {
        super(new ProfiledPIDController(
            Constants.KickerConstants.RIGHT_KICKER_PID.kP,
            Constants.KickerConstants.RIGHT_KICKER_PID.kI,
            Constants.KickerConstants.RIGHT_KICKER_PID.kD,
            Constants.KickerConstants.CONSTRAINTS
        ));
        configureKickerSpark();
    }

    public void configureKickerSpark() {
        rightKickerMotor.restoreFactoryDefaults();
        leftKickerMotor.restoreFactoryDefaults();

        rightKickerMotor.setInverted(KickerConstants.RIGHT_KICKER_INVERTED);
        leftKickerMotor.setInverted(KickerConstants.LEFT_KICKER_INVERTED);

        rightKickerMotor.setIdleMode(IdleMode.kBrake);
        leftKickerMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setVelocity(double kickerSpeed) {
        // TODO Use meters per second
        rightKickerMotor.set(kickerSpeed);
        leftKickerMotor.set(kickerSpeed);
    }

    public void stopKicker() {
        rightKickerMotor.set(0);
        leftKickerMotor.set(0);
    }

    public void resetEncoders() {
        rightKickerEncoder.setPosition(0);
        leftKickerEncoder.setPosition(0);
    }

    protected double getPositionDegrees() {
        return (rightKickerEncoder.getPosition() / leftKickerEncoder.getPosition() / Constants.KickerConstants.GEARING);
    }

    protected double getPositionRadians() {
        return Math.toRadians(getPositionDegrees());
    }

    protected double getPositionMeters() {
        return (getPositionRadians() * Units.inchesToMeters(2));
    }

    @Override
    protected double getMeasurement() {
        return getPositionMeters();
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        // TODO Use voltages and implement feedforward
    }
}