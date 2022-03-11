package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class OuterClimbThrottle extends CommandBase {
    private final Climb outerLeftClimb, outerRightClimb;
    private final DoubleSupplier throttleSupplier;

    public OuterClimbThrottle(Climb outerLeftClimb, Climb outerRightClimb, DoubleSupplier throttleSupplier) {
        this.outerLeftClimb = outerLeftClimb;
        this.outerRightClimb = outerRightClimb;
        this.throttleSupplier = throttleSupplier;

        addRequirements(outerLeftClimb, outerRightClimb);
    }

    @Override
    public void execute() {
        outerLeftClimb.setThrottle(throttleSupplier.getAsDouble());
        outerRightClimb.setThrottle(throttleSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        outerLeftClimb.setThrottle(0);
        outerRightClimb.setThrottle(0);
    }
}
