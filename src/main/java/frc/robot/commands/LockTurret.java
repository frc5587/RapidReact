package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

        // double sideBallTravel = shooter.timeOfFlight(distance) * drivetrain.getLinearVelocity() * Math.sin(turret.getPositionRadians() + error);
        // double angleAdjustment = Math.atan2(sideBallTravel, distance);
        // double totalError = error + angleAdjustment;
        
        Rotation2d clampedTurretAngle = new Rotation2d(MathUtil.clamp(desiredTurretAngle.getRadians(), turret.lowerLimit.getRadians(), turret.upperLimit.getRadians()));
        turret.setPose(clampedTurretAngle);
    }

    @Override
    public void end(boolean interrupted) {
        turret.setPose(new Rotation2d());
    }
}