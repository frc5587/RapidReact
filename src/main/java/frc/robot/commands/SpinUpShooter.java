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
        double distance = limelight.getDistanceToHub();
        double offAngle = limelight.getRelativeAngleToHub().getRadians();

        double speed = MathUtil.clamp(shooter.shootDistanceMoving(drivetrain.getLinearVelocity(), offAngle, distance), shooter.shootDistanceStationary(ShooterConstants.MIN_SHOOT_DISTANCE), shooter.shootDistanceStationary(ShooterConstants.MAX_SHOOT_DISTANCE));

        shooter.setVelocity(speed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.disable();
    }
}