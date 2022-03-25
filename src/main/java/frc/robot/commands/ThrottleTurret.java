package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;
import frc.robot.Constants.TurretConstants;

import java.util.function.DoubleSupplier;

/** Move the turret at a given throttle */
public class ThrottleTurret extends CommandBase {
    private final Turret turret;
    private final Limelight limelight;
    private final DoubleSupplier throttleSupplier;
    
    private boolean wasEnabledOnInit;
    private final double upperLimit = TurretConstants.LIMIT;
    private final double lowerLimit = -TurretConstants.LIMIT;

    public ThrottleTurret(Turret turret, Limelight limelight, DoubleSupplier throttleSupplier) {
        this.turret = turret;
        this.limelight = limelight;
        this.throttleSupplier = throttleSupplier;

        addRequirements(turret);
    }
    
    @Override
    public void initialize() {
        wasEnabledOnInit = turret.isEnabled();
        /** disable PID so it's not interfering with throttle control */
        turret.disable();
    }

    @Override
    public void execute() {
        /** 
         * if the turret is above the limit but the joystick is trying to move 
         * it back into range (the throttle is negative), let the turret move
         */
        System.out.println(throttleSupplier.getAsDouble());
        if(!limelight.hasTarget()) {
            if((turret.getPositionRadians() <= lowerLimit && throttleSupplier.getAsDouble() < 0) || (turret.getPositionRadians() >= upperLimit && throttleSupplier.getAsDouble() > 0)) {
                turret.setThrottle(-(throttleSupplier.getAsDouble() * TurretConstants.THROTTLE_MULTIPLIER));
            } else {
                turret.setThrottle(-(throttleSupplier.getAsDouble() * TurretConstants.THROTTLE_MULTIPLIER));
            }   
        } else {
            turret.stopTurret();
        }
    }    

    @Override
    public void end(boolean interrupted) {
        /* if the turret was enabled before the command was run, set it back to enabled;
         * otherwise, we can leave it disabled as it was supposed to be.
         */
        if(wasEnabledOnInit) {
            turret.enable();
        }
    }
}