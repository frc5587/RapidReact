package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/** Moves the turret to track the limelight target */
public class LockTurret extends CommandBase {
    private final Turret turret;
    private final Limelight limelight;
    private final Drivetrain drivetrain;
    private final Shooter shooter;

    public LockTurret(Turret turret, Limelight limelight, Drivetrain drivetrain, Shooter shooter) {
        this.turret = turret;
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.shooter = shooter;

        addRequirements(turret, limelight);
    }

    @Override
    public void initialize() {
        turret.enable();
    }

    @Override
    public void execute() {
        Rotation2d desiredTurretAngle = limelight.getRelativeAngleToHub();

        double distance = limelight.getDistanceToHub();
        Rotation2d offAngle = limelight.getRelativeAngleToHub();

        double sideBallTravel = shooter.timeOfFlight(distance) * drivetrain.getLinearVelocity() * offAngle.getSin();
        Rotation2d angleAdjustment = new Rotation2d(Math.atan2(sideBallTravel, distance));

        final double effect = 1;
        angleAdjustment = angleAdjustment.times(effect);
        desiredTurretAngle = desiredTurretAngle.plus(angleAdjustment);

        Rotation2d clampedTurretAngle = new Rotation2d(MathUtil.clamp(desiredTurretAngle.getRadians(), turret.lowerLimit.getRadians(), turret.upperLimit.getRadians()));
        turret.setPose(clampedTurretAngle);//,  new Rotation2d(drivetrain.getAngularVelocity() * .01));
    }

    @Override
    public void end(boolean interrupted) {
        turret.setPose(new Rotation2d());
    }
}