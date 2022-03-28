package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;
import frc.robot.Constants.ShooterConstants;

/** Spins up the shooter to the desired velocity and does not end */
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
        System.out.println("shooterspin!!!");
        if (limelight.hasTarget()) {
            shooter.setVelocity(shooter.shootDistanceStationary(limelight.calculateDistance()));

            // TODO, until we test these, imma leave them commented out
            // shooter.setVelocity(shooter.shootDistanceMoving(drivetrain.getLinearVelocity(), (turret.getPositionRadians() - limelight.getHorizontalAngle()), limelight.calculateDistance()));
            // System.out.println(shooter.shootDistanceMoving(drivetrain.getLinearVelocity(), (turret.getPositionRadians() - limelight.getHorizontalAngle()), limelight.calculateDistance()));
        } else {
            shooter.setVelocity(ShooterConstants.DEFAULT_SPIN_UP_VELOCITY);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.disable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}