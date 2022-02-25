package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeOut extends CommandBase {
    private final Intake intake;
    private final IntakePistons intakePistons;

    public IntakeOut(Intake intake, IntakePistons intakePistons) {
        this.intake = intake;
        this.intakePistons = intakePistons;
        addRequirements(intake, intakePistons);
    }

    @Override
    public void initialize() {
        intake.setVelocity(IntakeConstants.MIN_VELOCITY_REVERSE);
        intakePistons.extend();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVelocity(0);
        intakePistons.retract();
    }   
}
