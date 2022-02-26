package frc.robot.subsystems;

import frc.robot.Constants.ConveyorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.*;

public class Conveyor extends ProfiledPIDSubsystem {
    private static CANSparkMax conveyorMotor = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR, MotorType.kBrushless);
    private static RelativeEncoder encoder = conveyorMotor.getEncoder();

    private double velocitySetpoint = 0;
    public Conveyor() {
        super(new ProfiledPIDController(
            ConveyorConstants.PID.getP(),
            ConveyorConstants.PID.getI(),
            ConveyorConstants.PID.getD(),
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
        disable();
        velocitySetpoint = velocity;
    }

    public void setDistance(double distance) {
        setGoal(distance);
    }

    public void stop() {
        conveyorMotor.set(0);
    }

    public void resetEncoders() {
        encoder.setPosition(0);
    }

    @Override
    protected double getMeasurement() {
        return (encoder.getPosition() * 2 * Math.PI * ConveyorConstants.WHEEL_RADIUS / ConveyorConstants.GEARING);
    }

    protected double getVelocity() {
        return (encoder.getVelocity()/60 * 2 * Math.PI * ConveyorConstants.WHEEL_RADIUS / ConveyorConstants.GEARING);
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        if (setpoint.velocity != 0) {
            System.out.println("" + (ConveyorConstants.CONVEYOR_FF.calculate(setpoint.velocity) + output) + "  " + setpoint.velocity + "  " + getVelocity() + "  " + getMeasurement() + "  " + setpoint.position);
        
        }
        conveyorMotor.setVoltage(ConveyorConstants.CONVEYOR_FF.calculate(setpoint.velocity) );
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!isEnabled()) {
            conveyorMotor.setVoltage(ConveyorConstants.CONVEYOR_FF.calculate(velocitySetpoint) + ConveyorConstants.PID.calculate(velocitySetpoint - getVelocity()));
        }
    }
}