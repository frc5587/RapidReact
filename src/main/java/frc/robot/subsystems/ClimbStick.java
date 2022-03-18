package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ClimbConstants;

public class ClimbStick extends ProfiledPIDSubsystem {
    private ClimbController climbController;
    private MotorControllerGroup stickMotors;

    public ClimbStick(ProfiledPIDController PID, MotorControllerGroup stickMotors) {
        super(PID);
        this.stickMotors = stickMotors;
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        if(climbController.loaded) {
            stickMotors.setVoltage(ClimbConstants.UNLOADED_STICK_FF.calculate(setpoint.velocity));
        } else if (!climbController.loaded) {
            stickMotors.setVoltage(ClimbConstants.UNLOADED_STICK_FF.calculate(setpoint.velocity));
        }
        
    }

    @Override
    protected double getMeasurement() {
        return climbController.getStickPosition();
    }
}
