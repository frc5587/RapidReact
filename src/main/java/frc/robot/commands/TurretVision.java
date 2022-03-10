package frc.robot.commands;

import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurretVision extends CommandBase {
    private Turret turret;
    private Limelight limelight;
    private DoubleSupplier directionSupplier;

    public TurretVision(Turret turret, Limelight limelight, DoubleSupplier directionSupplier) {
        this.turret = turret;
        this.limelight = limelight;
        this.directionSupplier = directionSupplier;

        addRequirements(turret, limelight);
    }

    @Override
    public void initialize() {
        turret.enable();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        turret.setThrottle(0);
    }
}
