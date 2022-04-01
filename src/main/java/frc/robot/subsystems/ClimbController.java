package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimbConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.util.Units;

public class ClimbController extends SubsystemBase {
    private final CANSparkMax rightHookClimb = new CANSparkMax(ClimbConstants.RIGHT_HOOK_ARM_MOTOR, MotorType.kBrushless);
    private final CANSparkMax leftHookClimb = new CANSparkMax(ClimbConstants.LEFT_HOOK_ARM_MOTOR, MotorType.kBrushless);
    private final CANSparkMax rightStickClimb = new CANSparkMax(ClimbConstants.RIGHT_STICK_ARM_MOTOR, MotorType.kBrushless);
    private final CANSparkMax leftStickClimb = new CANSparkMax(ClimbConstants.LEFT_STICK_ARM_MOTOR, MotorType.kBrushless);

    private final RelativeEncoder rightHookEncoder = rightHookClimb.getEncoder();
    private final RelativeEncoder leftHookEncoder = leftHookClimb.getEncoder();
    private final RelativeEncoder rightStickEncoder = rightStickClimb.getEncoder();
    private final RelativeEncoder leftStickEncoder = leftStickClimb.getEncoder();

    private final MotorControllerGroup hookMotors = new MotorControllerGroup(rightHookClimb, leftHookClimb);
    private final MotorControllerGroup stickMotors = new MotorControllerGroup(rightStickClimb, leftStickClimb);

    protected boolean loaded;
    protected double hookPosition;
    protected double stickPosition;

    private ClimbHook climbHookLoaded = new ClimbHook(ClimbConstants.HOOK_LOADED_PID, this, hookMotors);
    private ClimbHook climbHookUnloaded = new ClimbHook(ClimbConstants.HOOK_UNLOADED_PID, this, hookMotors);
    private ClimbStick climbStickLoaded = new ClimbStick(ClimbConstants.STICK_LOADED_PID, this, stickMotors);
    private ClimbStick climbStickUnloaded = new ClimbStick(ClimbConstants.STICK_UNLOADED_PID, this, stickMotors);

    public ClimbController() {
        configureClimbMotors();
    }

    public void configureClimbMotors() {
        resetEncoders();

        rightHookClimb.restoreFactoryDefaults();
        leftHookClimb.restoreFactoryDefaults();
        rightStickClimb.restoreFactoryDefaults();
        leftStickClimb.restoreFactoryDefaults();

        rightHookClimb.setInverted(ClimbConstants.RIGHT_HOOK_MOTOR_INVERTED);
        leftHookClimb.setInverted(ClimbConstants.LEFT_HOOK_MOTOR_INVERTED);
        rightStickClimb.setInverted(ClimbConstants.RIGHT_STICK_ARM_MOTOR_INVERTED);
        leftStickClimb.setInverted(ClimbConstants.LEFT_STICK_ARM_MOTOR_INVERTED);

        rightHookClimb.setIdleMode(IdleMode.kBrake);
        leftHookClimb.setIdleMode(IdleMode.kBrake);
        rightStickClimb.setIdleMode(IdleMode.kBrake);
        leftStickClimb.setIdleMode(IdleMode.kBrake);

        rightHookClimb.setSmartCurrentLimit(ClimbConstants.STALL_CURRENT_LIMIT, ClimbConstants.FREE_CURRENT_LIMIT);
        leftHookClimb.setSmartCurrentLimit(ClimbConstants.STALL_CURRENT_LIMIT, ClimbConstants.FREE_CURRENT_LIMIT);
        rightStickClimb.setSmartCurrentLimit(ClimbConstants.STALL_CURRENT_LIMIT, ClimbConstants.FREE_CURRENT_LIMIT);
        leftStickClimb.setSmartCurrentLimit(ClimbConstants.STALL_CURRENT_LIMIT, ClimbConstants.FREE_CURRENT_LIMIT);
    }

    public void resetEncoders() {
        rightHookEncoder.setPosition(0);
        leftHookEncoder.setPosition(0);
        rightStickEncoder.setPosition(0);
        leftStickEncoder.setPosition(0);
    }

    /**
     * sets the hook motors to a given percentOutput throttle
     */
    public void setHookThrottle(double throttle) {
        hookMotors.set(throttle);
    }

    /**
     * sets the stick motors to a given percentOutput throttle
     */
    public void setStickThrottle(double throttle) {
        stickMotors.set(throttle);
    }

    /**
     * Indicates that climb has been loaded.
     */
    public void load() {
        loaded = true;
    }

    /**
     * Indicates that the climb has been unloaded.
     */
    public void unload() {
        loaded = false;
    }

    public void enable() {
        climbHookLoaded.enable();
        climbHookUnloaded.enable();
        climbStickLoaded.enable();
        climbStickUnloaded.enable();
    }

    public void disable() {
        climbHookLoaded.disable();
        climbHookUnloaded.disable();
        climbStickLoaded.disable();
        climbStickUnloaded.disable();
    }

    /**
     * @param position The desired hook position in meters
     */
    public void setHookPosition(double position) {
        hookPosition = position;
    }

    /**
     * @param position The desired stick position in meters
     */
    public void setStickPosition(double position) {
        stickPosition = position;
    }

    /**
     * @return Whether the climb is loaded.
     */
    public boolean isLoaded() {
        return loaded;
    }    
    
    public double getHookPosition() {
        return Units.rotationsToRadians(rightHookEncoder.getPosition()) * ClimbConstants.SPOOL_RADIUS / ClimbConstants.HOOK_GEARING;
    }

    public double getStickPosition() {
        return Units.rotationsToRadians(rightStickEncoder.getPosition()) * ClimbConstants.SPOOL_RADIUS / ClimbConstants.STICK_GEARING;
    }

    @Override
    public void periodic() {
        super.periodic();

        if(loaded) {
            climbHookLoaded.setGoal(hookPosition);
            climbStickLoaded.setGoal(stickPosition);
        } else {
            climbHookUnloaded.setGoal(hookPosition);
            climbStickUnloaded.setGoal(stickPosition);
        }
    }
}
