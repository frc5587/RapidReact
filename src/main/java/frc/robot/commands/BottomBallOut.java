package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Conveyor.ControlMode;

/** Ejects the bottom ball in the index out through the intake */
public class BottomBallOut extends CommandBase {
    private final Intake intake;
    private final IntakePistons intakePistons;
    private final Conveyor conveyor;

    public BottomBallOut(Intake intake, IntakePistons intakePistons, Conveyor conveyor) {
        this.intake = intake;
        this.intakePistons = intakePistons;
        this.conveyor = conveyor;

        addRequirements(intake, intakePistons, conveyor);
    }

    @Override
    public void initialize() {
        intakePistons.extend();

        conveyor.setControlMode(ControlMode.VELOCITY);
        conveyor.setVelocity(-2);

        intake.setVelocity(-3);
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        conveyor.setControlMode(ControlMode.OFF);

        intake.stop();
        intakePistons.retract();
    }
}
