package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleClimbPistons extends CommandBase {
    private final ClimbPistons climbPistons;

    public ToggleClimbPistons(ClimbPistons climbPistons) {
        this.climbPistons = climbPistons;

        addRequirements(climbPistons);
    }

    @Override
    public void initialize() {
        if(climbPistons.pistonSet1.get() == Value.kReverse) {
            climbPistons.pistonSet1.set(Value.kForward);
            climbPistons.pistonSet2.set(Value.kReverse);
        } else {
            climbPistons.pistonSet1.set(Value.kReverse);
            climbPistons.pistonSet2.set(Value.kForward);
        }

    }
}
