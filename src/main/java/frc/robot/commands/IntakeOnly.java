package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeOnly extends CommandBase {
    private final Intake intake;
    private final IntakePistons intakePistons;

    public IntakeOnly(Intake intake, IntakePistons intakePistons) {
        this.intake = intake;
        this.intakePistons = intakePistons;

        addRequirements(intake, intakePistons);
    }

    /*
    Extend the intake before doing anything
    */
    @Override
    public void initialize() {
        // Extend the intake
        intakePistons.extend();
        // Set the intake velocity
        intake.setVelocity(3);
    }

    @Override
    public void end(boolean interrupted) {
        intakePistons.retract();
        intake.stop();
    }
}
