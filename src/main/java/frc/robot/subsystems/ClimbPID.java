package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

public class ClimbPID extends SubsystemBase {
    private Climb climbLoaded = new Climb(new ProfiledPIDController(
        ClimbConstants.HOOK_LOADED_PID.getP(),
        ClimbConstants.HOOK_LOADED_PID.getI(),
        ClimbConstants.HOOK_LOADED_PID.getD(),
        ClimbConstants.CONSTRAINTS)
    );
    private Climb climbUnloaded = new Climb(new ProfiledPIDController(
        ClimbConstants.HOOK_UNLOADED_PID.getP(),
        ClimbConstants.HOOK_UNLOADED_PID.getI(),
        ClimbConstants.HOOK_UNLOADED_PID.getD(),
        ClimbConstants.CONSTRAINTS)
    );

    public ClimbState climbState = ClimbState.OFF;

    public ClimbPID() {
    }

    public enum ClimbState {
        OFF,
        LOADED,
        UNLOADED
    }

    public void setState(ClimbState climbState) {
        this.climbState = climbState;
    }

    public void setPosition(double position) {
        if(climbState == ClimbState.OFF) {
            climbUnloaded.setGoal(0);
        } else if (climbState == ClimbState.UNLOADED) {
            climbUnloaded.setGoal(position);
        } else if (climbState == ClimbState.LOADED) {
            climbLoaded.setGoal(position);
        }
    }
}
