package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;

public class Kicker extends ProfiledPIDSubsystem {
    private final CANSparkMax kickerMotor;
    private final RelativeEncoder kickerEncoder;
    private final SimpleMotorFeedforward feedforward;
    private final boolean motorInverted;
    private State lastSetpoint = new State(0, 0);

    public Kicker(int motorId, SimpleMotorFeedforward feedForward, ProfiledPIDController PID, boolean motorInverted) {
        super(PID);

        this.feedforward = feedForward;
        this.motorInverted = motorInverted;

        kickerMotor = new CANSparkMax(motorId, MotorType.kBrushless);
        kickerEncoder = kickerMotor.getEncoder();
        
        configureKickerSpark();
    }

    private void configureKickerSpark() {
        resetEncoders();
        kickerMotor.restoreFactoryDefaults();
        kickerMotor.setInverted(motorInverted);
        kickerMotor.setIdleMode(IdleMode.kBrake);
        kickerMotor.setSmartCurrentLimit(KickerConstants.STALL_CURRENT_LIMIT, KickerConstants.FREE_CURRENT_LIMIT);
    }

    public void moveDistance(double distance) {
        setGoal(distance);
    }

    public void stop() {
        kickerMotor.set(0);
    }

    public void resetEncoders() {
        kickerEncoder.setPosition(0);
    }

    public double getPosition() {
        return (Units.rotationsToRadians(kickerEncoder.getPosition()) * (KickerConstants.WHEEL_RADIUS / KickerConstants.GEARING));
    }

    public void moveMore(double distance) {
        moveDistance(getPosition() + distance);
    }

    @Override
    protected double getMeasurement() {
        return getPosition();
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        double ff = feedforward.calculate(setpoint.velocity);
        lastSetpoint = setpoint;

        kickerMotor.setVoltage(ff + output);
    }

    public boolean isDone() {
        return (lastSetpoint.velocity == 0);
    }

    public static Kicker createRightKicker() {
        return new Kicker(KickerConstants.RIGHT_KICKER_MOTOR, KickerConstants.RIGHT_KICKER_FF, KickerConstants.RIGHT_KICKER_PID, KickerConstants.RIGHT_KICKER_INVERTED);
    }

    public static Kicker createLeftKicker() {
        return new Kicker(KickerConstants.LEFT_KICKER_MOTOR, KickerConstants.LEFT_KICKER_FF, KickerConstants.LEFT_KICKER_PID, KickerConstants.LEFT_KICKER_INVERTED);
    }
}