package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Manually sets the climb subsystems to given throttles. */
public class ClimbThrottle extends CommandBase {
    private final ClimbController climb;
    private final Turret turret;
    private final IntakePistons intakePistons;
    private final DoubleSupplier throttleSupplier, stickThrottleSupplier;
    private final BooleanSupplier throttleToggleSupplier;

    public ClimbThrottle(ClimbController climb, Turret turret, IntakePistons intakePistons, DoubleSupplier throttleSupplier, DoubleSupplier stickThrottleSupplier, BooleanSupplier throttleToggleSupplier) {
        this.climb = climb;
        this.turret = turret;
        this.intakePistons = intakePistons;
        this.throttleSupplier = throttleSupplier;
        this.stickThrottleSupplier = stickThrottleSupplier;
        this.throttleToggleSupplier = throttleToggleSupplier;

        addRequirements(climb, turret, intakePistons);
    }

    @Override
    public void initialize() {
        climb.disable();
        turret.enable();
        turret.setPose(new Rotation2d());
    }

    @Override
    public void execute() {
        double hookThrottle = throttleSupplier.getAsDouble();
        
        if (hookThrottle < 0) {
            intakePistons.extend();
            hookThrottle *= .4;
        } else if (hookThrottle > 0) {
            intakePistons.extend();
        }

        if (throttleToggleSupplier.getAsBoolean()) {
            climb.setHookThrottle(-1);
        } else {
            climb.setHookThrottle(hookThrottle);
        }

        SmartDashboard.putNumber("Hook Throttle", throttleSupplier.getAsDouble());
        SmartDashboard.putNumber("Hook Set", hookThrottle);
        climb.setStickThrottle(-stickThrottleSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        climb.setHookThrottle(0);
        climb.setStickThrottle(0);
        intakePistons.retract();
    }
}
