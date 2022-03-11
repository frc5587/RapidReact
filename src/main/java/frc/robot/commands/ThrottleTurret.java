package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Turret;

public class ThrottleTurret extends CommandBase {
    private final DoubleSupplier positionSupplier;
    private final Turret turret;

    public ThrottleTurret(Turret turret, DoubleSupplier positionSupplier) {
        this.positionSupplier = positionSupplier;
        this.turret = turret;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.enable();
    }

    @Override
    public void execute() {
        double movePercent = positionSupplier.getAsDouble();
        double position = TurretConstants.LIMIT * movePercent;

        turret.setPosition(position);
    }
}