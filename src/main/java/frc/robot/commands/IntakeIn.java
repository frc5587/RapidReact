package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.*;

import frc.robot.subsystems.Conveyor.ControlMode;

public class IntakeIn extends CommandBase {
    private final Intake intake;
    private final IntakePistons intakePistons;
    private final Conveyor conveyor;

    public IntakeIn(Intake intake, IntakePistons intakePistons, Conveyor conveyor) {
        this.intake = intake;
        this.intakePistons = intakePistons;
        this.conveyor = conveyor;
        addRequirements(intake, intakePistons, conveyor);
    }

    @Override
    public void initialize() {
        intake.setVelocity(IntakeConstants.MIN_VELOCITY_FORWARD);
        intakePistons.extend();

        conveyor.setControlMode(Conveyor.ControlMode.VELOCITY);
        conveyor.setVelocity(ConveyorConstants.VELOCITY_FORWARD);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVelocity(0);
        intakePistons.retract();
        conveyor.setVelocity(0);

        conveyor.setControlMode(ControlMode.OFF);
    }
}
