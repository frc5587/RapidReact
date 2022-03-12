package frc.robot.subsystems;

import frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;

public class Climb extends ProfiledPIDSubsystem {
    private CANSparkMax climbMotor;
    private RelativeEncoder climbEncoder;
    private ElevatorFeedforward feedforward;
    private boolean motorInverted;
    private State lastSetpoint = new State(0, 0);
    private boolean isLoaded;

    public Climb(int motorId, ElevatorFeedforward feedforward, PIDController PID, boolean motorInverted) {
        super(new ProfiledPIDController(
            PID.getP(), 
            PID.getI(), 
            PID.getD(), 
            ClimbConstants.CONSTRAINTS
        ));
        
        this.feedforward = feedforward;
        this.motorInverted = motorInverted;

        climbMotor = new CANSparkMax(motorId, MotorType.kBrushless);
        climbEncoder = climbMotor.getEncoder();

        configureClimbMotors();
    }

    public void configureClimbMotors() {
        resetEncoders();
        climbMotor.restoreFactoryDefaults();
        climbMotor.setInverted(motorInverted);
        climbMotor.setIdleMode(IdleMode.kBrake);

        // // sets the soft limits of the motors in meters
        // climbMotor.setSoftLimit(SoftLimitDirection.kForward, (float)ClimbConstants.SOFT_LIMITS[1]);
        // climbMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)ClimbConstants.SOFT_LIMITS[0]);
        // climbMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        // climbMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    public double getPosition() {
        return (Units.rotationsToRadians(climbEncoder.getPosition()) * (ClimbConstants.SPOOL_RADIUS / ClimbConstants.GEARING));
    }

    public void resetEncoders() {
        climbEncoder.setPosition(0);
    }

    @Override
    protected double getMeasurement() {
        return getPosition();
    }

    public void setPosition(double position, boolean isLoaded) {
        if (position < ClimbConstants.LOWER_LIMIT || position > ClimbConstants.UPPER_lIMIT) {
            System.out.println(position + " is out of bounds for climb");
        } else {
            this.isLoaded = isLoaded;
            setGoal(position);
        }
    }

    public void positionThrottle(double throttle) {
        setPosition(getPosition() + ClimbConstants.CONSTRAINTS.maxVelocity * 2 * 0.02 * throttle, false);
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        if (getPosition() < ClimbConstants.LOWER_LIMIT || getPosition() > ClimbConstants.UPPER_lIMIT) {
            if ((setpoint.velocity < 0 && getPosition() < ClimbConstants.LOWER_LIMIT) || (setpoint.velocity > 0 && getPosition() > ClimbConstants.UPPER_lIMIT)) {
                return; // do nothing,
            }
        }
        double ff = 0;
        if (isLoaded) {
            ff = ClimbConstants.LOADED_ELEVATOR_FF.calculate(setpoint.velocity);
        } else {
            ff = ClimbConstants.UNLOADED_ELEVATOR_FF.calculate(setpoint.velocity);
        }
        lastSetpoint = setpoint;

        climbMotor.setVoltage(ff + output);
    }

    public static Climb createInnerRightArm() {
        return new Climb(ClimbConstants.INNER_CLIMB_RIGHT_MOTOR, ClimbConstants.LOADED_ELEVATOR_FF, ClimbConstants.UNLOADED_ELEVATOR_PID, ClimbConstants.INNER_CLIMB_LEFT_MOTOR_INVERTED);
    }

    public static Climb createOuterRightArm() {
        return new Climb(ClimbConstants.OUTER_CLIMB_RIGHT_MOTOR, ClimbConstants.LOADED_ELEVATOR_FF, ClimbConstants.UNLOADED_ELEVATOR_PID, ClimbConstants.INNER_CLIMB_LEFT_MOTOR_INVERTED);
    }

    public static Climb createInnerLeftArm() {
        return new Climb(ClimbConstants.INNER_CLIMB_LEFT_MOTOR, ClimbConstants.LOADED_ELEVATOR_FF, ClimbConstants.UNLOADED_ELEVATOR_PID, ClimbConstants.INNER_CLIMB_LEFT_MOTOR_INVERTED);
    }

    public static Climb createOuterLeftArm() {
        return new Climb(ClimbConstants.OUTER_CLIMB_LEFT_MOTOR, ClimbConstants.LOADED_ELEVATOR_FF, ClimbConstants.UNLOADED_ELEVATOR_PID, ClimbConstants.INNER_CLIMB_LEFT_MOTOR_INVERTED);
    }
}