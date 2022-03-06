package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.KickerConstants;

import com.revrobotics.RelativeEncoder;

public class Kicker extends ProfiledPIDSubsystem {
    private CANSparkMax kickerMotor;
    private RelativeEncoder kickerEncoder;
    private SimpleMotorFeedforward feedforward;
    private boolean motorInverted;
    private State lastSetpoint = new State(0, 0);

    public Kicker(int motorId, SimpleMotorFeedforward feedForward, PIDController PID, boolean motorInverted) {
        super(new ProfiledPIDController(
            PID.getP(),
            PID.getI(),
            PID.getD(),
            Constants.KickerConstants.CONSTRAINTS
        ));

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
        return Units.rotationsToRadians(kickerEncoder.getPosition()) * KickerConstants.WHEEL_RADIUS / KickerConstants.GEARING;
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