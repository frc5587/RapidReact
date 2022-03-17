package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;

public class SpinUpShooter extends CommandBase {
    private final Shooter shooter;
    private final Drivetrain drivetrain;
    private final Turret turret;
    private final Limelight limelight;

    public SpinUpShooter(Shooter shooter, Drivetrain drivetrain, Turret turret, Limelight limelight) {
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.turret = turret;
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
            shooter.setVelocity(shooter.shootDistanceMoving(drivetrain, turret, limelight, limelight.calculateDistance()));
            System.out.println(shooter.shootDistanceMoving(drivetrain, turret, limelight, limelight.calculateDistance()));
        } else {
            shooter.setVelocity(ShooterConstants.DEFAULT_SPIN_UP_VELOCITY);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.disable();
    }
}