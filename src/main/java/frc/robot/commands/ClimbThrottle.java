package frc.robot.commands;

import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbThrottle extends CommandBase {
    private final ClimbTest climbTest;
    private final DoubleSupplier throttleSupplier;

    public ClimbThrottle(ClimbTest climbTest, DoubleSupplier throttleSupplier) {
        this.climbTest = climbTest;
        this.throttleSupplier = throttleSupplier;

        addRequirements(climbTest);
    }

    @Override
    public void execute() {
        climbTest.setThrottle(throttleSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        climbTest.setThrottle(0);
    }
}
