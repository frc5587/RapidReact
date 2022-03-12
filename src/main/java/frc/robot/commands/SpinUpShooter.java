package frc.robot.commands;

import java.io.Console;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class SpinUpShooter extends CommandBase {
    private final Shooter shooter;
    private final Limelight limelight;

    public SpinUpShooter(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.enable();
    }

    @Override
    public void execute() {
        if (limelight.hasTarget()) {
            shooter.setVelocity(shooter.shootDistance(limelight.calculateDistance()));
            System.out.println(shooter.shootDistance(limelight.calculateDistance()));
        } else {
            shooter.setVelocity(ShooterConstants.DEFAULT_SPIN_UP_VELOCITY);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.disable();
    }
}