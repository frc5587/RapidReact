package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunThrottle extends CommandBase{
    public DoubleSupplier throttleSupplier;
    public Intake intake;

    public RunThrottle(Intake intake, DoubleSupplier throttleSupplier) {
        this.throttleSupplier = throttleSupplier;
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.moveWithThrottle(throttleSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
