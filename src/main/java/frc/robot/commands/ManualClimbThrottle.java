package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class ManualClimbThrottle extends CommandBase {
    private Climb innerRightClimb, innerLeftClimb;
    private DoubleSupplier throttleSupplier;

    @Deprecated()
    public ManualClimbThrottle(Climb innerRightClimb, Climb innerLeftClimb, DoubleSupplier throttleSupplier) {
        this.innerRightClimb = innerRightClimb;
        this.innerLeftClimb = innerLeftClimb;
        this.throttleSupplier = throttleSupplier;

        addRequirements(innerRightClimb, innerLeftClimb);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        innerRightClimb.setThrottle(throttleSupplier.getAsDouble());
        innerLeftClimb.setThrottle(throttleSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        innerRightClimb.setThrottle(0);
        innerLeftClimb.setThrottle(0);
    }
}
