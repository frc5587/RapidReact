package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.*;

/** Moves the turret to track the limelight target */
public class LockTurret extends CommandBase {
    private final Turret turret;
    private final Limelight limelight;
    private final Drivetrain drivetrain;
    private final Shooter shooter;

    private final Notifier limelightNotifier = new Notifier(this::updateLimelight);
    private final int updateRate = 90; // framerate of limelight 

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

        limelightNotifier.startPeriodic(1.0/updateRate);
    }

    public void updateLimelight() {
        if (limelight.hasTarget()) {
            double distance = limelight.calculateDistance();
            Rotation2d angleError = new Rotation2d(-limelight.getHorizontalAngleRadians());
            double timeStamp = limelight.calculateFPGAFrameTimestamp();
            Rotation2d drivetrainAngle = drivetrain.getHeadingAtTime(timeStamp);

            Rotation2d relativeAnglePosition = turret.getPosition().plus(angleError).plus(drivetrainAngle).plus(Rotation2d.fromDegrees(180));
            Translation2d estimatedOffset = new Translation2d(distance * relativeAnglePosition.getCos(), distance * relativeAnglePosition.getSin());
            Pose2d estimatedPosition = new Pose2d(estimatedOffset.plus(TurretConstants.HUB_POSITION), drivetrainAngle); 

            drivetrain.updateOdometryEstimatorWithVision(estimatedPosition, timeStamp);
        }
    }

    @Override
    public void execute() {
        // if(limelight.hasTarget()) {
        //     double error = -limelight.getHorizontalAngleRadians(); 
        //     double distance = limelight.calculateDistance();
        //     double sideBallTravel = shooter.timeOfFlight(distance) * drivetrain.getLinearVelocity() * Math.sin(turret.getPositionRadians() + error);
        //     double angleAdjustment = Math.atan2(sideBallTravel, distance);
        //     double totalError = error + angleAdjustment;
        //     if (Math.abs(totalError) > 0.05) { //TODO: try changing this number to make turret motion better
        //         turret.setPosition(turret.getPositionRadians() + totalError);
        //         // , -drivetrain.getAngularVelocity());
        //     }
        // }

        Translation2d relativePosition = drivetrain.getEstimatedPose().getTranslation().minus(TurretConstants.HUB_POSITION);
        Rotation2d angleToHub = new Rotation2d(relativePosition.getX(), relativePosition.getY());
        Rotation2d relativeAngleToRobot = angleToHub.plus(Rotation2d.fromDegrees(180)).plus(angleToHub);

        turret.setPose(relativeAngleToRobot);
    }

    @Override
    public void end(boolean interrupted) {
        turret.setPose(new Rotation2d());
    }
}