package frc.robot.subsystems;

import frc.robot.Constants.ConveyorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.*;
import edu.wpi.first.math.util.Units;

public class Conveyor extends ProfiledPIDSubsystem {
    private static CANSparkMax conveyorMotor = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR, MotorType.kBrushless);
    private static RelativeEncoder encoder = conveyorMotor.getEncoder();

    public Conveyor() {
        super(new ProfiledPIDController(
            ConveyorConstants.PID.kP,
            ConveyorConstants.PID.kI,
            ConveyorConstants.PID.kD,
            ConveyorConstants.CONSTRAINTS
        ));
        configureConveyorSpark();
    }

    public void configureConveyorSpark() {
        resetEncoders();

        conveyorMotor.restoreFactoryDefaults();

        conveyorMotor.setInverted(ConveyorConstants.MOTOR_INVERTED);

        conveyorMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setVelocity(double velocity) {
        setGoal(new State(0, velocity));
    }

    public void stop() {
        conveyorMotor.set(0);
    }

    public void resetEncoders() {
        encoder.setPosition(0);
    }

    @Override
    protected double getMeasurement() {
        return (encoder.getVelocity() / 60 * 2 * Math.PI * Units.inchesToMeters(2) / ConveyorConstants.GEARING);
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        conveyorMotor.setVoltage(ConveyorConstants.CONVEYOR_FF.calculate(setpoint.velocity - output));
    }
}