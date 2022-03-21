package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.*;

public class ToggleIntakePistons extends InstantCommand {
    private final IntakePistons intakePistons;

    public ToggleIntakePistons(IntakePistons intakePistons) {
        this.intakePistons = intakePistons;

        addRequirements(intakePistons);
    }

    @Override
    public void initialize() {
        if(intakePistons.isExtended()) {
            intakePistons.retract();
        } else {
            intakePistons.extend();
        }
    }
}
