package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbController;

public class MakeClimbCoast extends CommandBase {
    private final ClimbController climbController;

    public MakeClimbCoast(ClimbController climbController) {
        this.climbController = climbController;

        addRequirements(climbController);
    }

    @Override
    public void initialize() {
        climbController.setIdleMode(false);
    }

    @Override
    public void end(boolean interrupted) {
        climbController.setIdleMode(true);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
