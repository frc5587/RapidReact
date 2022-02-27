package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeOut extends CommandBase {
    private final Intake intake;
    private final IntakePistons intakePistons;
    private final Conveyor conveyor;

    public IntakeOut(Intake intake, IntakePistons intakePistons, Conveyor conveyor) {
        this.intake = intake;
        this.intakePistons = intakePistons;
        this.conveyor = conveyor;
        addRequirements(intake, intakePistons, conveyor);
    }

    @Override
    public void initialize() {
        intake.setVelocity(IntakeConstants.MIN_VELOCITY_REVERSE);
        intakePistons.extend();

        conveyor.setControlMode(Conveyor.ControlMode.VELOCITY);
        conveyor.setVelocity(ConveyorConstants.VELOCITY_REVERSE);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVelocity(0);
        intakePistons.retract();
        conveyor.setVelocity(0);
    }   
}
