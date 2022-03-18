package frc.robot.commands;

import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TankDrive extends CommandBase {
    private Drivetrain drivetrain;
    private DoubleSupplier leftThrottleSupplier, rightThrottleSupplier;

    public TankDrive(Drivetrain drivetrain, DoubleSupplier leftThrottleSupplier, DoubleSupplier rightThrottleSupplier) {
        this.drivetrain = drivetrain;
        this.leftThrottleSupplier = leftThrottleSupplier;
        this.rightThrottleSupplier = rightThrottleSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double leftThrottle = leftThrottleSupplier.getAsDouble();
        double rightThrottle = rightThrottleSupplier.getAsDouble();
        drivetrain.tankDriveVolts(leftThrottle, rightThrottle);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
