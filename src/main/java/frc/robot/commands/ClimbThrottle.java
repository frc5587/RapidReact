package frc.robot.commands;

import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbThrottle extends CommandBase {
    private ClimbController climb;
    private DoubleSupplier hookThrottle, stickThrottle;

    public ClimbThrottle(ClimbController climb, DoubleSupplier hookThrottle, DoubleSupplier stickThrottle) {
        this.climb = climb;
        this.hookThrottle = hookThrottle;
        this.stickThrottle = stickThrottle;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.disable();
    }

    @Override
    public void execute() {
        climb.setHookThrottle(hookThrottle.getAsDouble());
        climb.setStickThrottle(stickThrottle.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        climb.setHookThrottle(0);
        climb.setStickThrottle(0);
    }
}
