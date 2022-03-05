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
    private double velocitySetpoint = 0;
    private ControlMode controlMode = ControlMode.OFF;

    public enum ControlMode {
        POSITION, VELOCITY, OFF
    }

    public Conveyor() {
        super(new ProfiledPIDController(
            ConveyorConstants.POSITION_PID.getP(),
            ConveyorConstants.POSITION_PID.getI(),
            ConveyorConstants.POSITION_PID.getD(),
            ConveyorConstants.CONSTRAINTS
        ));

        configureConveyorSpark();
    }

    private void configureConveyorSpark() {
        resetEncoders();
        conveyorMotor.restoreFactoryDefaults();
        conveyorMotor.setInverted(ConveyorConstants.MOTOR_INVERTED);
        conveyorMotor.setIdleMode(IdleMode.kBrake);
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

    @Override
    protected void useOutput(double output, State setpoint) {
        // if (setpoint.velocity != 0) {
        //     System.out.println("" + (ConveyorConstants.CONVEYOR_FF.calculate(setpoint.velocity) + output) + "  " + setpoint.velocity + "  " + getVelocity() + "  " + getMeasurement() + "  " + setpoint.position);
        // }
        conveyorMotor.setVoltage(ConveyorConstants.CONVEYOR_FF.calculate(setpoint.velocity) );
    }

    @Override
    public void periodic() {
        super.periodic();
        System.out.println(getPosition() + "  " + getVelocity());

        if (controlMode == ControlMode.VELOCITY) {
            conveyorMotor.setVoltage(ConveyorConstants.CONVEYOR_FF.calculate(velocitySetpoint) + ConveyorConstants.VELOCITY_PID.calculate(velocitySetpoint));
        }
    }
}