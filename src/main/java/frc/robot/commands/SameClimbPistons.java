package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SameClimbPistons extends CommandBase {
    private final ClimbPistons climbPistons;
    private final boolean isForward;

    public SameClimbPistons(ClimbPistons climbPistons, boolean isForward) {
        this.climbPistons = climbPistons;
        this.isForward = isForward;

        addRequirements(climbPistons);
    }

    @Override
    public void initialize() {
        if (isForward) {
            climbPistons.extendSet1();
            climbPistons.extendSet2();
        } else {
            climbPistons.retractSet1();
            climbPistons.retractSet2();
        }
    }
}
