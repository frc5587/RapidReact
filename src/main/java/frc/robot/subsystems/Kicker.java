package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants.KickerConstants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class Kicker extends ProfiledPIDSubsystem {
    private static CANSparkMax kickerMotorLeader = new CANSparkMax(KickerConstants.KICKER_MOTOR_LEADER, MotorType.kBrushless);
    private static CANSparkMax kickerMotorFollower = new CANSparkMax(KickerConstants.KICKER_MOTOR_FOLLOWER, MotorType.kBrushless);

    private static RelativeEncoder kickerEncoderMain = kickerMotorLeader.getEncoder();
    private static RelativeEncoder kickerEncoder2 = kickerMotorFollower.getEncoder();
    
    public Kicker() {
        super(new ProfiledPIDController(
            Constants.KickerConstants.PID.kP,
            Constants.KickerConstants.PID.kI,
            Constants.KickerConstants.PID.kD,
            Constants.KickerConstants.CONSTRAINTS
        ));
        configureKickerSpark();
    }



    public void configureKickerSpark() {
        kickerMotorLeader.restoreFactoryDefaults();
        kickerMotorFollower.restoreFactoryDefaults();

        kickerMotorLeader.setInverted(KickerConstants.LEADER_INVERTED);

        kickerMotorLeader.setIdleMode(IdleMode.kCoast);
        kickerMotorFollower.setIdleMode(IdleMode.kCoast);
    }

    public void setKicker(double kickerSpeed) {
        kickerMotorLeader.setIdleMode(IdleMode.kCoast);
        kickerMotorFollower.setIdleMode(IdleMode.kCoast);

        kickerMotorLeader.set(-kickerSpeed);
        kickerMotorFollower.set(-kickerSpeed);
    }

    public void stopKicker() {
        kickerMotorLeader.set(0);
        kickerMotorFollower.set(0);

        kickerMotorLeader.setIdleMode(IdleMode.kBrake);
        kickerMotorFollower.setIdleMode(IdleMode.kBrake);
    }

    public void resetEncoders() {
        kickerEncoderMain.setPosition(0);
        kickerEncoder2.setPosition(0);
    }

    protected double getPositionDegrees() {
        return (kickerEncoderMain.getPosition() / kickerEncoder2.getPosition() / Constants.KickerConstants.GEARING / Constants.KickerConstants.ENCODER_CPR);
    }

    protected double getPositionRadians() {
        return Math.toRadians(getPositionDegrees());
    }

    @Override
    protected double getMeasurement() {
        return getPositionRadians();
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        System.out.println("Test later");
    }
}