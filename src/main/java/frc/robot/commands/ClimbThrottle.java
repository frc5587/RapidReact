package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Manually sets the climb subsystems to given throttles. */
public class ClimbThrottle extends CommandBase {
    private final ClimbController climb;
    private final Turret turret;
    private final DoubleSupplier throttleSupplier, stickThrottleSupplier;
    private final BooleanSupplier throttleToggleSupplier;
    private boolean isHookMode = true;
    private NetworkTableEntry toggleEntry = SmartDashboard.getEntry("Hook Mode Enabled");
    private SlewRateLimiter hookThrottleLimiter = new SlewRateLimiter(0.5); // TODO: tune this

    public ClimbThrottle(ClimbController climb, Turret turret, DoubleSupplier throttleSupplier, DoubleSupplier stickThrottleSupplier, BooleanSupplier throttleToggleSupplier) {
        this.climb = climb;
        this.turret = turret;
        this.throttleSupplier = throttleSupplier;
        this.stickThrottleSupplier = stickThrottleSupplier;
        this.throttleToggleSupplier = throttleToggleSupplier;

        addRequirements(climb, turret);
    }

    @Override
    public void initialize() {
        climb.disable();
        turret.enable();
        turret.setPose(new Rotation2d());
    }

    @Override
    public void execute() {
        // if (throttleToggleSupplier.getAsBoolean()) {
        //     isHookMode = !isHookMode;
        // }

        // if (isHookMode) {
        //     climb.setHookThrottle(throttleSupplier.getAsDouble());
        // } else {
        //     climb.setStickThrottle(throttleSupplier.getAsDouble());
        //     climb.setHookThrottle(ClimbConstants.STEADY_STATE_HOOK_THROTTLE);
        // }
        climb.setHookThrottle(hookThrottleLimiter.calculate(throttleSupplier.getAsDouble()));
        climb.setStickThrottle(stickThrottleSupplier.getAsDouble());

        updateSmartDashboard();

    }

    @Override
    public void end(boolean interrupted) {
        climb.setHookThrottle(0);
        climb.setStickThrottle(0);
    }

    public void updateSmartDashboard() {
        toggleEntry.setBoolean(isHookMode);
    }
}
