package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class ClimbThrottle extends CommandBase {
    private final Climb climb1, climb2, climb3, climb4;
    private final DoubleSupplier throttleSupplier;

    public ClimbThrottle(Climb climb1, Climb climb2, Climb climb3, Climb climb4, DoubleSupplier throttleSupplier) {
        this.climb1 = climb1;
        this.climb2 = climb2;
        this.climb3 = climb3;
        this.climb4 = climb4;
        this.throttleSupplier = throttleSupplier;

        addRequirements(climb1, climb2, climb3, climb4);
    }

    @Override
    public void execute() {
        climb1.throttle(throttleSupplier.getAsDouble());
        climb2.throttle(throttleSupplier.getAsDouble());
        climb3.throttle(throttleSupplier.getAsDouble());
        climb4.throttle(throttleSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        climb1.throttle(0);
        climb2.throttle(0);
        climb3.throttle(0);
        climb4.throttle(0);
    }
}
