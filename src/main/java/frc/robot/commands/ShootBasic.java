package frc.robot.commands;

import frc.robot.subsystems.*;

// import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootBasic extends CommandBase {
    private Shooter shooter;
    private double throttleSupplier;

    public ShootBasic(Shooter shooter, double throttleSupplier) {
        this.shooter = shooter;
        this.throttleSupplier = throttleSupplier;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setVelocity(throttleSupplier);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setVelocity(0);
        shooter.resetEncoders();
    }
}
