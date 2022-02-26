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
import frc.robot.Constants.KickerConstants;

import com.revrobotics.RelativeEncoder;

public class Kicker extends ProfiledPIDSubsystem {
    private CANSparkMax kickerMotor;
    private RelativeEncoder kickerEncoder;
    private SimpleMotorFeedforward feedforward;
    private boolean motorInverted;
    private int motorId;
    private PIDController PID;

    public Kicker(int motorId, SimpleMotorFeedforward feedForward, PIDController PID, boolean motorInverted) {
        super(new ProfiledPIDController(
            PID.getP(),
            PID.getI(),
            PID.getD(),
            Constants.KickerConstants.CONSTRAINTS
        ));

        this.motorId = motorId;
        this.feedforward = feedForward;
        this.PID = PID;
        this.motorInverted = motorInverted;

        kickerMotor = new CANSparkMax(motorId, MotorType.kBrushless);
        kickerEncoder = kickerMotor.getEncoder();
        
        configureKickerSpark();
    }

    public void configureKickerSpark() {
        resetEncoders();

        kickerMotor.restoreFactoryDefaults();

        kickerMotor.setInverted(motorInverted);

        kickerMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setDistance(double distance) {
        setGoal(distance);
    }

    public void stop() {
        kickerMotor.set(0);
    }

    public void resetEncoders() {
        kickerEncoder.setPosition(0);
    }

    @Override
    protected double getMeasurement() {
        return (kickerEncoder.getPosition() * (2 * Math.PI) * KickerConstants.WHEEL_RADIUS / KickerConstants.GEARING);
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        kickerMotor.setVoltage(feedforward.calculate(setpoint.velocity) + output);
    }

    public static Kicker createRightKicker() {
        return new Kicker(KickerConstants.RIGHT_KICKER_MOTOR, KickerConstants.RIGHT_KICKER_FF, KickerConstants.RIGHT_KICKER_PID, KickerConstants.RIGHT_KICKER_INVERTED);
    }

    public static Kicker createLeftKicker() {
        return new Kicker(KickerConstants.LEFT_KICKER_MOTOR, KickerConstants.LEFT_KICKER_FF, KickerConstants.LEFT_KICKER_PID, KickerConstants.LEFT_KICKER_INVERTED);
    }

    public boolean isSpinning() {
        return (kickerEncoder.getVelocity() > 0.01);
    }
}