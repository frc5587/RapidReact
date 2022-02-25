package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeIn extends CommandBase {
    private final Intake intake;
    private final IntakePistons intakePistons;

    public IntakeIn(Intake intake, IntakePistons intakePistons) {
        this.intake = intake;
        this.intakePistons = intakePistons;
        addRequirements(intake, intakePistons);
    }

    @Override
    public void execute() {
        intake.setVelocity(3);
        intakePistons.extend();
        intake.enable();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        intakePistons.retract();
    }
}
