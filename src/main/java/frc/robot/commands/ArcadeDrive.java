package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;

import edu.wpi.first.math.filter.SlewRateLimiter;
import java.util.function.DoubleSupplier;


public class ArcadeDrive extends CommandBase {
    private final Drivetrain drivetrain;
    private final DoubleSupplier throttleSupplier, curveSupplier;
    private SlewRateLimiter throttleFilter = new SlewRateLimiter(4);
    private SlewRateLimiter curveFilter = new SlewRateLimiter(4);

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

        drivetrain.arcadeDrive(throttleFilter.calculate(throttle), curveFilter.calculate(curve));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}