package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class ClimbThrottle extends CommandBase {
    private final Climb climb1, climb2;
    private final DoubleSupplier throttleSupplier;

    public ClimbThrottle(Climb climb1, Climb climb2, DoubleSupplier throttleSupplier) {
        this.climb1 = climb1;
        this.climb2 = climb2;
        this.throttleSupplier = throttleSupplier;

        addRequirements(climb1, climb2);
    }

    @Override
    public void execute() {
        climb1.positionThrottle(throttleSupplier.getAsDouble());
        climb2.positionThrottle(throttleSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        climb1.positionThrottle(0);
        climb2.positionThrottle(0);
    }
}
