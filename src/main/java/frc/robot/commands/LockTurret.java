package frc.robot.commands;

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

    public void updateLimelight() {
        if (limelight.hasTarget()) {
            Rotation2d angleError = new Rotation2d(-limelight.getHorizontalAngleRadians());
            if (Math.abs(angleError.getDegrees()) < 4 ) {
                // double distance = limelight.calculateDistance();
                // double timeStamp = limelight.calculateFPGAFrameTimestamp();
                // Rotation2d drivetrainAngle = drivetrain.getHeadingAtTime(timeStamp);

                // Rotation2d relativeAnglePosition = turret.getPosition().plus(angleError).plus(drivetrainAngle).plus(Rotation2d.fromDegrees(180));
                // Translation2d estimatedOffset = new Translation2d(distance * relativeAnglePosition.getCos(), distance * relativeAnglePosition.getSin());
                // Pose2d estimatedPosition = new Pose2d(estimatedOffset.plus(TurretConstants.HUB_POSITION), drivetrainAngle); 

                // System.out.println(estimatedPosition + "   " + timeStamp + "  " + Timer.getFPGATimestamp() + "  " + (Timer.getFPGATimestamp()-(limelight.pipelineLatencyMS()/1000)));

                // drivetrain.updateOdometryEstimatorWithVision(estimatedPosition, Timer.getFPGATimestamp()-(limelight.pipelineLatencyMS()/1000));
            }
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

        // Translation2d relativePosition = drivetrain.getEstimatedPose().getTranslation().minus(TurretConstants.HUB_POSITION);
        // Rotation2d angleToHub = new Rotation2d(relativePosition.getX(), relativePosition.getY());
        // Rotation2d relativeAngleToRobot = angleToHub.plus(Rotation2d.fromDegrees(180)).plus(angleToHub);

        Translation2d target = limelight.getUpperHubPosition();
        Pose2d robotPose = drivetrain.getPose();
        
        Translation2d positionDifference = robotPose.getTranslation().minus(target);
        Rotation2d robotToTargetAngle = new Rotation2d(positionDifference.getX(), positionDifference.getY());
        Rotation2d desiredTurretAngle = robotToTargetAngle.minus(robotPose.getRotation());

        // Rotation2d angleError = new Rotation2d(-limelight.getHorizontalAngleRadians());
        // Rotation2d turnAngle = turret.getPosition().plus(angleError);
        // System.out.println(angleError + "  " + turnAngle + "   " + turret.getPosition() );
        turret.setPose(desiredTurretAngle);
    }

    @Override
    public void end(boolean interrupted) {
        turret.setPose(new Rotation2d());
    }
}