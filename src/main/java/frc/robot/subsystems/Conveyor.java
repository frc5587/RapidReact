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
    private static CANSparkMax conveyorMotorMain = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR, MotorType.kBrushless);
    private static RelativeEncoder encoderMain = conveyorMotorMain.getEncoder();

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
        conveyorMotorMain.restoreFactoryDefaults();

        conveyorMotorMain.setInverted(ConveyorConstants.MOTOR_INVERTED);

        conveyorMotorMain.setIdleMode(IdleMode.kBrake);
    }

    public void setConveyor(double conveyorSpeed) {
        conveyorMotorMain.set(conveyorSpeed);
    }

    public void stopConveyor() {
        conveyorMotorMain.set(0);
    }

    public void resetEncoders() {
        encoderMain.setPosition(0);
    }

    protected double getPositionDegrees() {
        return (encoderMain.getPosition() / Constants.ConveyorConstants.GEARING / Constants.ConveyorConstants.ENCODER_CPR);
    }

    protected double getPositionRadians() {
        return Math.toRadians(getPositionDegrees());
    }

    // TODO Only use meters

    @Override
    protected double getMeasurement() {
        return getPositionRadians();
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        // TODO Use voltages and implement feedforward
    }
}