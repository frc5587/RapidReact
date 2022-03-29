package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import java.util.function.DoubleSupplier;

/** Manually sets the climb subsystems to given throttles. */
public class ClimbThrottle extends CommandBase {
    private final ClimbController climb;
    private final Turret turret;
    private final DoubleSupplier hookThrottle, stickThrottle;

    public boolean isClimbing;

    public ClimbThrottle(ClimbController climb, Turret turret, DoubleSupplier hookThrottle, DoubleSupplier stickThrottle) {
        this.climb = climb;
        this.turret = turret;
        this.hookThrottle = hookThrottle;
        this.stickThrottle = stickThrottle;

        addRequirements(climb, turret);
    }

    @Override
    public void initialize() {
        isClimbing = true;
        climb.disable();
        turret.enable();
        turret.setPose(new Rotation2d());
        turret.disable();
    }

    @Override
    public void execute() {
        climb.setHookThrottle(hookThrottle.getAsDouble());
        climb.setStickThrottle(stickThrottle.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        isClimbing = false;
        climb.setHookThrottle(0);
        climb.setStickThrottle(0);
    }
}
