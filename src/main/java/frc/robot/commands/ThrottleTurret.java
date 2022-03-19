package frc.robot.commands;

import org.frc5587.lib.control.DeadbandXboxController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Turret;

public class ThrottleTurret extends CommandBase {
    // private final DoubleSupplier positionSupplier;
    private final Turret turret;
    private final DeadbandXboxController xb; 
    private final double slowness = 0.1;

    public ThrottleTurret(Turret turret, DeadbandXboxController xb) {
        this.xb = xb;
        this.turret = turret;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.enable();
    }

    @Override
    public void execute() {
        double movePercent = xb.getLeftX();
        double position = TurretConstants.LIMIT * movePercent * slowness;

        turret.setPosition(position);
    }

    
}