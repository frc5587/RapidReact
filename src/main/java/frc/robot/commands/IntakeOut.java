package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class IntakeOut extends CommandBase {
    private final Intake intake;
    private final IntakePistons intakePistons;

    public IntakeOut(Intake intake, IntakePistons intakePistons) {
        this.intake = intake;
        this.intakePistons = intakePistons;
        addRequirements(intake, intakePistons);
    }

    @Override
    public void execute() {
        intake.setVelocity(-2);
        intakePistons.extend();
        intake.enable();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        intakePistons.retract();
    }   
}
