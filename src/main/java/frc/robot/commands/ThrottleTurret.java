package frc.robot.commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;
import frc.robot.Constants.TurretConstants;

import java.util.function.DoubleSupplier;

import org.frc5587.lib.advanced.ObjectTracker;

/** Move the turret at a given throttle */
public class ThrottleTurret extends CommandBase {
    private final Turret turret;
    private final Limelight limelight;
    private final DoubleSupplier throttleSupplier;
    
    private boolean wasEnabledOnInit;
    private final double upperLimit = TurretConstants.LIMIT;
    private final double lowerLimit = -TurretConstants.LIMIT;

    private final Notifier limelightNotifier = new Notifier(this::updateLimelight);
    private final int updateRate = 90; // framerate of limelight 
    private final ObjectTracker upperHubTracker = new ObjectTracker();

    private final double moveAmount = 0.5;

    public ThrottleTurret(Turret turret, Limelight limelight, DoubleSupplier throttleSupplier) {
        this.turret = turret;
        this.limelight = limelight;
        this.throttleSupplier = throttleSupplier;

        limelightNotifier.startPeriodic(1.0/updateRate);

        addRequirements(turret);
    }

    private void updateLimelight() {

    }
    
    @Override
    public void initialize() {
        wasEnabledOnInit = turret.isEnabled();
        /** disable PID so it's not interfering with throttle control */
        turret.enable();
    }

    @Override
    public void execute() {
        turret.setPosition(turret.getPositionRadians() + (moveAmount * throttleSupplier.getAsDouble()));
    } 
}