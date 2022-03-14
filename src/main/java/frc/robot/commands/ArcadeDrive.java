package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import org.frc5587.lib.MathHelper;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class ArcadeDrive extends CommandBase {
    private final Drivetrain drivetrain;
    private final DoubleSupplier throttleSupplier, curveSupplier;
    private double lastThrottle = 0;

    public ArcadeDrive(Drivetrain drivetrain, DoubleSupplier throttleSupplier, DoubleSupplier curveSupplier) {
        this.drivetrain = drivetrain;
        this.throttleSupplier = throttleSupplier;
        this.curveSupplier = curveSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double throttle = throttleSupplier.getAsDouble();
        double curve = curveSupplier.getAsDouble();

        lastThrottle += Math.min(Math.max(throttle - lastThrottle, -1./15.), 1./15.);

        drivetrain.arcadeDrive(lastThrottle, curve);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}