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
    private final IntakePistons intakePistons;
    private final DoubleSupplier throttleSupplier, stickThrottleSupplier;
    private final BooleanSupplier throttleToggleSupplier;
    private boolean isHookMode = true;
    private NetworkTableEntry toggleEntry = SmartDashboard.getEntry("Hook Mode Enabled");
    private SlewRateLimiter hookThrottleLimiter = new SlewRateLimiter(0); 

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
        } else if (hookThrottle > 0) {
            intakePistons.retract();
        }
        
        climb.setHookThrottle(hookThrottle);
        climb.setStickThrottle(-stickThrottleSupplier.getAsDouble());

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
