package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants.ClimbConstants;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class ClimbStick extends ProfiledPIDSubsystem {
    private final ClimbController climbController;
    private final MotorControllerGroup stickMotors;

    public ClimbStick(ProfiledPIDController PID, ClimbController climbController, MotorControllerGroup stickMotors) {
        super(PID);
        this.climbController = climbController;
        this.stickMotors = stickMotors;
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        if(climbController.loaded) {
            stickMotors.setVoltage(ClimbConstants.LOADED_STICK_FF.calculate(setpoint.velocity));
        } else {
            stickMotors.setVoltage(ClimbConstants.UNLOADED_STICK_FF.calculate(setpoint.velocity));
        }
        
    }

    @Override
    protected double getMeasurement() {
        return climbController.getStickPosition();
    }
}
