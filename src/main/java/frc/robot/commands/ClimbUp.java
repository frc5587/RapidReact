package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.subsystems.ClimbPID.ClimbState;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbUp extends CommandBase {
    private ClimbPID climbPID;
    private Climb climb;

    public ClimbUp(ClimbPID climbPID, Climb climb) {
        this.climbPID = climbPID;
        this.climb = climb;

        addRequirements(climbPID);
    }

    @Override
    public void initialize() {
        climb.enable();
        climbPID.setState(ClimbState.UNLOADED);
    }

    @Override
    public void execute() {
        climbPID.setPosition(1);
    }
    
    @Override
    public void end(boolean interrupted) {
        climbPID.setState(ClimbState.OFF);
    }
}
