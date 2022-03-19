package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Turret;

public class ThrottleTurret extends CommandBase {
    private final Turret turret;
    private final DoubleSupplier throttleSupplier;
    private boolean wasEnabledOnInit;

    public ThrottleTurret(Turret turret, DoubleSupplier throttleSupplier) {
        this.turret = turret;
        this.throttleSupplier = throttleSupplier;

        addRequirements(turret);
    }
    @Override
    public void initialize() {
        wasEnabledOnInit = turret.isEnabled();
        turret.disable();
    }

    @Override
    public void execute() {
        if(Math.abs(turret.getPositionRadians()) >= TurretConstants.LIMIT) {
            turret.stopTurret();
        }
        else {
            turret.setThrottle(throttleSupplier.getAsDouble() * TurretConstants.THROTTLE_MULTIPLIER);
        }
    }    

    @Override
    public void end(boolean interrupted) {
        // if the turret was enabled before the command was run, set it back to enabled;
        // otherwise, we can leave it disabled as it was supposed to be.
        if(wasEnabledOnInit) {
            turret.enable();
        }
    }
}