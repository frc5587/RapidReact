package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretThrottle extends CommandBase{
    public DoubleSupplier throttleSupplier;
    public Turret turret;

    public TurretThrottle(Turret turret, DoubleSupplier throttleSupplier) {
        this.throttleSupplier = throttleSupplier;
        this.turret = turret;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        turret.setThrottle(throttleSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        turret.stopTurret();
    }
}