package frc.robot.commands;

import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootBasic extends CommandBase {
    private Shooter shooter;
    private DoubleSupplier throttleSupplier;

    public ShootBasic(Shooter shooter, DoubleSupplier throttleSupplier) {
        this.shooter = shooter;
        this.throttleSupplier = throttleSupplier;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setThrottle(throttleSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setThrottle(0);
        shooter.resetEncoders();
    }
}
