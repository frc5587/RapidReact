package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Conveyor.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

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
        // Extend the intake
        intakePistons.extend();

        // Move the conveyor backwards at a constant velocity.
        conveyor.setControlMode(ControlMode.VELOCITY);
        conveyor.setVelocity(-1);

        // Move the intake backwards at a constant velocity.
        intake.setVelocity(-3);
    }

    /*
    When the command ends, stop the conveyor & intake, then retract the intake.
    */
    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        conveyor.setControlMode(ControlMode.OFF);

        intake.stop();
        intakePistons.retract();
    }
}
