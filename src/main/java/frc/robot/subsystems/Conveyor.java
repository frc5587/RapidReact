package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.trajectory.TrapezoidProfile.*;
import edu.wpi.first.math.util.Units;

/**
Use a motor to control wheels in order to intake & outtake balls to the robot.
*/
public class Conveyor extends ProfiledPIDSubsystem {
    private final CANSparkMax conveyorMotor = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder encoder = conveyorMotor.getEncoder();
    private double velocitySetpoint = 0;
    private ControlMode controlMode = ControlMode.OFF;

    public enum ControlMode {
        POSITION, VELOCITY, OFF
    }

    public Conveyor() {
        super(ConveyorConstants.POSITION_PID);

        configureConveyorSpark();
    }

    private void configureConveyorSpark() {
        resetEncoders();
        conveyorMotor.restoreFactoryDefaults();
        conveyorMotor.setInverted(ConveyorConstants.MOTOR_INVERTED);
        conveyorMotor.setIdleMode(IdleMode.kBrake);

        conveyorMotor.setSmartCurrentLimit(ConveyorConstants.STALL_CURRENT_LIMIT, ConveyorConstants.FREE_CURRENT_LIMIT);
    }    

    public void setControlMode(ControlMode controlMode) {
        this.controlMode = controlMode;

        if (controlMode == ControlMode.OFF || controlMode == ControlMode.VELOCITY) {
            disable();
        } else {
            enable();
        }
    }

    public ControlMode getControlMode() {
        return controlMode;
    }

    public void setVelocity(double velocity) {
        if (controlMode == ControlMode.VELOCITY) {
            velocitySetpoint = velocity;
        } else {
            throw new RuntimeException("Cannot control velocity, control mode is on: " + controlMode);
        }
    }

    public void moveDistance(double distance) {
        if (controlMode == ControlMode.POSITION) {
            setGoal(distance);
        } else {
            throw new RuntimeException("Cannot control position, control mode is on: " + controlMode);
        }
    }

    public void moveMore(double distance) {
        moveDistance(getPosition() + distance);
    }

    public void stop() {
        conveyorMotor.set(0);
    }

    public void resetEncoders() {
        encoder.setPosition(0);
    }

    @Override
    protected double getMeasurement() {
        return getPosition();
    }

    public double getPosition() {
        return Units.rotationsToRadians(encoder.getPosition()) * ConveyorConstants.WHEEL_RADIUS / ConveyorConstants.GEARING;
    }

    public double getVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) * ConveyorConstants.WHEEL_RADIUS / ConveyorConstants.GEARING;
    }

    public boolean hasBall() {
        // TODO Check if velocity slows/voltage decreases when ball is in conveyor. Use this to return a boolean on if the conveyor has a ball.
        return false;
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        conveyorMotor.setVoltage(ConveyorConstants.CONVEYOR_FF.calculate(setpoint.velocity));
    }

    @Override
    public void periodic() {
        super.periodic();

        if (controlMode == ControlMode.VELOCITY) {
            conveyorMotor.setVoltage(ConveyorConstants.CONVEYOR_FF.calculate(velocitySetpoint) + ConveyorConstants.VELOCITY_PID.calculate(velocitySetpoint));
        }
    }
}