package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;

/** Move the turret at a given throttle */
public class ThrottleTurret extends CommandBase {
    private final Turret turret;
    private final DoubleSupplier throttleSupplier;

    private final double multiplier = 0.25;

    public ThrottleTurret(Turret turret, Limelight limelight, DoubleSupplier throttleSupplier) {
        this.turret = turret;
        this.throttleSupplier = throttleSupplier;

        addRequirements(turret);
    }
    
    @Override
    public void initialize() {
        turret.enable();
    }

    @Override
    public void execute() {
        turret.setVelocity(new Rotation2d(throttleSupplier.getAsDouble() * multiplier));
    } 
}