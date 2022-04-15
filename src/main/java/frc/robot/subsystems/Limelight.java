package frc.robot.subsystems;

import org.frc5587.lib.subsystems.LimelightBase;

import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends LimelightBase {
    private final Drivetrain drivetrain;
    private final Turret turret;
    private final Notifier limelightNotifier = new Notifier(this::updateTarget);
    private final int updateRate = 85; // just less than the framerate of limelight
    private Translation2d upperHub = new Translation2d(LimelightConstants.HUB_POSITION.getX(), LimelightConstants.HUB_POSITION.getY());

    public Limelight(Drivetrain drivetrain, Turret turret) {
        super(LimelightConstants.MOUNT_ANGLE, LimelightConstants.LENS_HEIGHT_METERS, 
                LimelightConstants.GOAL_HEIGHT_METERS, LimelightConstants.DISTANCE_OFFSET);
                
        this.drivetrain = drivetrain;
        this.turret = turret;

        limelightNotifier.startPeriodic(1.0/updateRate);
    }

    private void updateTarget() {
        if (hasTarget() && Math.abs(getHorizontalAngleRadians()) < Units.degreesToRadians(20)) {
            Rotation2d angleError = new Rotation2d(-getHorizontalAngleRadians());
            double distance = calculateDistance();
            double timeStamp = calculateFPGAFrameTimestamp();
            Pose2d poseAtTime = drivetrain.getPoseAtTime(timeStamp);

            if (poseAtTime == null) {
                return; // needs to run longer to get odometry info
            }

            Rotation2d relativeAnglePosition = turret.getPosition().plus(angleError).plus(poseAtTime.getRotation()).plus(Rotation2d.fromDegrees(180));
            Translation2d estimatedOffset = new Translation2d(distance * relativeAnglePosition.getCos(), distance * relativeAnglePosition.getSin());

            Translation2d estimatedPastHubPosition = poseAtTime.getTranslation().minus(estimatedOffset);
            Translation2d estimatedCurrentHubPosition = estimatedPastHubPosition.plus(drivetrain.getPose().minus(poseAtTime).getTranslation());

            upperHub = upperHub.plus(estimatedCurrentHubPosition).div(2); // average the two together
        }
    }

    public Translation2d getUpperHubPosition() {
        return upperHub;
    }

    public Rotation2d getAngleAroundHubTheRobotIs() {
        Pose2d robotPose = drivetrain.getPose();
        Translation2d positionDifference = robotPose.getTranslation().minus(upperHub);
        Rotation2d robotToTargetAngle = new Rotation2d(positionDifference.getX(), positionDifference.getY());
        
        return robotToTargetAngle;
    }

    public Rotation2d getRelativeAngleToHub() {
        Pose2d robotPose = drivetrain.getPose();

        return getAngleAroundHubTheRobotIs().minus(robotPose.getRotation()).minus(Rotation2d.fromDegrees(180));
    }

    public double getDistanceToHub() {
        return upperHub.getDistance(drivetrain.getPose().getTranslation());
    }

    public boolean hasAcceptableHorizontalError(double radiansError) {
        return Math.abs(getHorizontalAngleRadians()) < radiansError;
    }

    @Override
    public void periodic() {
        Translation2d offset = LimelightConstants.HUB_POSITION.minus(upperHub);
        drivetrain.updateFieldWidget(offset);

        SmartDashboard.putNumber("Predicted distance", getDistanceToHub());
        SmartDashboard.putNumber("Limelight Distance", calculateDistance());
        SmartDashboard.putBoolean("Has Target", hasTarget());
    }
}
