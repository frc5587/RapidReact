package frc.robot.subsystems;

import frc.robot.Constants.*;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climb extends ProfiledPIDSubsystem {
    private CANSparkMax rightClimb = new CANSparkMax(ClimbConstants.RIGHT_HOOK_ARM_MOTOR, MotorType.kBrushless);
    private CANSparkMax leftClimb = new CANSparkMax(ClimbConstants.LEFT_HOOK_ARM_MOTOR, MotorType.kBrushless);
    private CANSparkMax stickClimb = new CANSparkMax(ClimbConstants.STICK_ARM_MOTOR, MotorType.kBrushless);

    private MotorControllerGroup hookMotors = new MotorControllerGroup(rightClimb, leftClimb);

    public Climb(ProfiledPIDController PID) {
        super(new ProfiledPIDController(
            PID.getP(),
            PID.getI(),
            PID.getD(),
            ClimbConstants.CONSTRAINTS
        ));
        configureClimbMotors();
    }

    public void configureClimbMotors() {
        rightClimb.restoreFactoryDefaults();
        leftClimb.restoreFactoryDefaults();
        stickClimb.restoreFactoryDefaults();

        rightClimb.setInverted(ClimbConstants.RIGHT_HOOK_MOTOR_INVERTED);
        leftClimb.setInverted(ClimbConstants.LEFT_HOOK_MOTOR_INVERTED);
        stickClimb.setInverted(ClimbConstants.STICK_ARM_MOTOR_INVERTED);

        rightClimb.setIdleMode(IdleMode.kBrake);
        leftClimb.setIdleMode(IdleMode.kBrake);
        stickClimb.setIdleMode(IdleMode.kBrake);

        rightClimb.setSmartCurrentLimit(ClimbConstants.STALL_CURRENT_LIMIT, ClimbConstants.FREE_CURRENT_LIMIT);
        leftClimb.setSmartCurrentLimit(ClimbConstants.STALL_CURRENT_LIMIT, ClimbConstants.FREE_CURRENT_LIMIT);
        stickClimb.setSmartCurrentLimit(ClimbConstants.STALL_CURRENT_LIMIT, ClimbConstants.FREE_CURRENT_LIMIT);
    }

    public void setHookThrottle(double throttle) {
        hookMotors.set(throttle);
    }

    public void setStickThrottle(double throttle) {
        stickClimb.set(throttle);
    }

    @Override
    protected void useOutput(double output, State setpoint) {
    }

    @Override
    protected double getMeasurement() {
        return 0;
    }
}
