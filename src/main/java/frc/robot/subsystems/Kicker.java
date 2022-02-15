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
    private static CANSparkMax kickerMotorMain = new CANSparkMax(KickerConstants.KICKER_MOTOR_MAIN, MotorType.kBrushless);
    private static CANSparkMax kickerMotor2 = new CANSparkMax(KickerConstants.KICKER_MOTOR_2, MotorType.kBrushless);

    private static RelativeEncoder kickerEncoderMain = kickerMotorMain.getEncoder();
    private static RelativeEncoder kickerEncoder2 = kickerMotor2.getEncoder();

    // private static MotorControllerGroup motorControllerGroup = new MotorControllerGroup(kickerMotorMain, kickerMotor2);

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
        kickerMotorMain.restoreFactoryDefaults();
        kickerMotor2.restoreFactoryDefaults();

        kickerMotorMain.setInverted(KickerConstants.MOTOR_INVERTED);
        kickerMotor2.setInverted(KickerConstants.MOTOR_INVERTED);

        kickerMotorMain.setIdleMode(IdleMode.kCoast);
        kickerMotor2.setIdleMode(IdleMode.kCoast);
    }

    public void setKicker(double kickerSpeed) {
        kickerMotorMain.setIdleMode(IdleMode.kCoast);
        kickerMotor2.setIdleMode(IdleMode.kCoast);

        kickerMotorMain.set(-kickerSpeed);
        kickerMotor2.set(-kickerSpeed);
    }

    public void stopKicker() {
        kickerMotorMain.set(0);
        kickerMotor2.set(0);

        kickerMotorMain.setIdleMode(IdleMode.kBrake);
        kickerMotor2.setIdleMode(IdleMode.kBrake);
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