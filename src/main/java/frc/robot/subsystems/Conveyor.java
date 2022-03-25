package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants.ConveyorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;

/**
Use a motor to control wheels in order to intake & outtake balls to the robot.
*/
public class Conveyor extends ProfiledPIDSubsystem {
    private final CANSparkMax conveyorMotor = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder encoder = conveyorMotor.getEncoder();
    private double velocitySetpoint = 0;
    private ControlMode controlMode = ControlMode.OFF;

    /** 
     * The control mode of the conveyor, used to change PID usage
     * <p>
     * <ul>
     * <li><b>ControlMode</b>
     * <ul>
     * <li> <b>POSITION:</b> allows the conveyor to be controlled by setting a position </li>
     * <li> <b>VELOCITY:</b> allows the conveyor to be controlled by setting a velocity </li>
     * <li> <b>OFF:</b> a default value that disables control altogether. </li>
     * </ul></li>
     * </ul>
     */
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
        conveyorMotor.setSmartCurrentLimit(ConveyorConstants.STALL_CURRENT_LIMIT,
                ConveyorConstants.FREE_CURRENT_LIMIT);
    }    

    /**
     * Sets the control mode and handles PID enabled state
     * @param controlMode the desired {@link ControlMode} to use with the conveyor
     */
    public void setControlMode(ControlMode controlMode) {
        this.controlMode = controlMode;

        if (controlMode == ControlMode.OFF || controlMode == ControlMode.VELOCITY) {
            disable();
        } else {
            enable();
        }
    }

    /**
     * @return the current {@link ControlMode} of the conveyor
     */
    public ControlMode getControlMode() {
        return controlMode;
    }

    /**
     * Sets the setpoint as a velocity if ControlMode velocity is chosen
     * @param velocity the desired velocity in radians per second
     */
    public void setVelocity(double velocity) {
        if (controlMode == ControlMode.VELOCITY) {
            velocitySetpoint = velocity;
        } else {
            throw new RuntimeException("Cannot control velocity, control mode is on: " + controlMode);
        }
    }

    /**
     * Sets the setpoint as a position if ControlMode position is chosen
     * @param distance the desired position in radians
     */
    public void setPosition(double position) {
        if (controlMode == ControlMode.POSITION) {
            setGoal(position);
        } else {
            throw new RuntimeException("Cannot control position, control mode is on: " + controlMode);
        }
    }

    /**
     * Adds a desired distance to the current position (moves the conveyor by a given amount)
     * @param distance the distance to move the conveyor in radians
     */
    public void moveDistance(double distance) {
        setPosition(getPosition() + distance);
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