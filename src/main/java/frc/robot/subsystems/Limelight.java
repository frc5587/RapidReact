package frc.robot.subsystems;

import org.frc5587.lib.subsystems.LimelightBase;

import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends LimelightBase {
    private final Drivetrain drivetrain;
    private final Turret turret;
    private final Notifier limelightNotifier = new Notifier(this::updateTarget);
    private final int updateRate = 85; // just less than the framerate of limelight
    private Translation2d upperHub = LimelightConstants.HUB_POSITION;

    public Limelight(Drivetrain drivetrain, Turret turret) {
        super(LimelightConstants.MOUNT_ANGLE, LimelightConstants.LENS_HEIGHT_METERS, 
                LimelightConstants.GOAL_HEIGHT_METERS, LimelightConstants.DISTANCE_OFFSET);
                
        this.drivetrain = drivetrain;
        this.turret = turret;

        limelightNotifier.startPeriodic(1.0/updateRate);
    }

    private void updateTarget() {
        if (hasTarget()) {
            Rotation2d angleError = new Rotation2d(-getHorizontalAngleRadians());
            double distance = calculateDistance();
            double timeStamp = calculateFPGAFrameTimestamp();
            Pose2d poseAtTime = drivetrain.getPoseAtTime(timeStamp);

            Rotation2d relativeAnglePosition = turret.getPosition().plus(angleError).plus(poseAtTime.getRotation()).plus(Rotation2d.fromDegrees(180));
            Translation2d estimatedOffset = new Translation2d(distance * relativeAnglePosition.getCos(), distance * relativeAnglePosition.getSin());
            // Pose2d estimatedPosition = new Pose2d(estimatedOffset.plus(TurretConstants.HUB_POSITION), drivetrainAngle); 
            Translation2d estimatedPastHubPosition = poseAtTime.getTranslation().minus(estimatedOffset);
            Translation2d estimatedCurrentHubPosition = estimatedPastHubPosition.plus(drivetrain.getPose().minus(poseAtTime).getTranslation());

            upperHub = upperHub.plus(estimatedCurrentHubPosition).div(2); // average the two together

            // System.out.println(estimatedPosition + "   " + timeStamp + "  " + Timer.getFPGATimestamp() + "  " + (Timer.getFPGATimestamp()-(limelight.pipelineLatencyMS()/1000)));

            // drivetrain.updateOdometryEstimatorWithVision(estimatedPosition, Timer.getFPGATimestamp()-(limelight.pipelineLatencyMS()/1000));
        }

    }

    public Translation2d getUpperHubPosition() {
        return upperHub;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Limelight Distance", calculateDistance());
        SmartDashboard.putBoolean("Has Target", hasTarget());
    }
}
