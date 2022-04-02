package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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
        if (limelight.hasTarget()) {
            // shooter.setVelocity(shooter.shootDistanceStationary(limelight.calculateDistance()));
            double speed = MathUtil.clamp(shooter.shootDistanceMoving(drivetrain.getLinearVelocity(), (turret.getPosition().getRadians() - limelight.getHorizontalAngleRadians()), limelight.calculateDistance()), shooter.shootDistanceStationary(ShooterConstants.MIN_SHOOT_DISTANCE), shooter.shootDistanceStationary(ShooterConstants.MAX_SHOOT_DISTANCE));

            // TODO, until we test these, imma leave them commented out
            System.out.println(speed + "  " + shooter.shootDistanceStationary(limelight.calculateDistance()));
            shooter.setVelocity(speed);
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