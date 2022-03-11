package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class InnerClimbThrottle extends CommandBase {
    private final Climb innerLeftClimb, innerRightClimb;
    private final DoubleSupplier throttleSupplier;

    public InnerClimbThrottle(Climb innerLeftClimb, Climb innerRightClimb, DoubleSupplier throttleSupplier) {
        this.innerLeftClimb = innerLeftClimb;
        this.innerRightClimb = innerRightClimb;
        this.throttleSupplier = throttleSupplier;

        addRequirements(innerLeftClimb, innerRightClimb);
    }

    @Override
    public void execute() {
        innerLeftClimb.setThrottle(throttleSupplier.getAsDouble());
        innerRightClimb.setThrottle(throttleSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        innerLeftClimb.setThrottle(0);
        innerRightClimb.setThrottle(0);
    }
}
