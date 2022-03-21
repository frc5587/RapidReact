package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants.*;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class ClimbHook extends ProfiledPIDSubsystem {
    private final ClimbController climbController;
    private final MotorControllerGroup hookMotors;

    public ClimbHook(ProfiledPIDController PID, ClimbController climbController, MotorControllerGroup hookMotors) {
        super(PID);
        this.climbController = climbController;
        this.hookMotors = hookMotors;
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        if(climbController.loaded) {
            hookMotors.setVoltage(ClimbConstants.LOADED_HOOK_FF.calculate(setpoint.velocity));
        } else if (!climbController.loaded) {
            hookMotors.setVoltage(ClimbConstants.UNLOADED_HOOK_FF.calculate(setpoint.velocity));
        }
    }

    @Override
    protected double getMeasurement() {
        return climbController.getHookPosition();
    }
}
