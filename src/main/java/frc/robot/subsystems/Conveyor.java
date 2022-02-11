package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants.ConveyorConstants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class Conveyor extends ProfiledPIDSubsystem {
    private static CANSparkMax conveyorMotor = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR, MotorType.kBrushless);
    private static RelativeEncoder encoder = conveyorMotor.getEncoder();

    public Conveyor() {
        super(new ProfiledPIDController(
            Constants.ConveyorConstants.PID.kP,
            Constants.ConveyorConstants.PID.kI,
            Constants.ConveyorConstants.PID.kD,
            Constants.ConveyorConstants.CONSTRAINTS
        ));
        configureConveyorSpark();
    }

    public void configureConveyorSpark() {
        conveyorMotor.restoreFactoryDefaults();

        conveyorMotor.setInverted(ConveyorConstants.MOTOR_INVERTED);

        conveyorMotor.setIdleMode(IdleMode.kCoast);
    }

    public void setConveyor(double conveyorSpeed) {
        conveyorMotor.setIdleMode(IdleMode.kCoast);
        conveyorMotor.set(-conveyorSpeed);
    }

    public void stopConveyor() {
        conveyorMotor.set(0);
        conveyorMotor.setIdleMode(IdleMode.kBrake);
    }

    public void resetEncoders() {
        encoder.setPosition(0);
    }

    protected double getPositionDegrees() {
        return (encoder.getPosition() / Constants.ConveyorConstants.GEARING / Constants.ConveyorConstants.ENCODER_CPR);
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