package frc.robot.commands;

import java.util.ArrayList;

import org.frc5587.lib.auto.RamseteCommandWrapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class GotoHubSpot extends CommandBase {
    private final Drivetrain drivetrain;
    private final Limelight limelight;

    private final double ideal = Units.inchesToMeters(120);
    private final double acceptableError = Units.inchesToMeters(10);

    private RamseteCommandWrapper ramseteCommand;

    public GotoHubSpot(Drivetrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d start = drivetrain.getPose();
        Translation2d hub = limelight.getUpperHubPosition();

        double distance = hub.getDistance(start.getTranslation());
        Rotation2d finalAngle = limelight.getAngleAroundHubTheRobotIs();
        Rotation2d relativeAngle = limelight.getRelativeAngleToHub();



        Pose2d endPoint = new Pose2d(distance * finalAngle.getCos(), distance * finalAngle.getSin(), finalAngle.minus(Rotation2d.fromDegrees(180)));

        // if (distance < ideal - acceptableError) {
        //     if (relativeAngle.getDegrees() < 90 && relativeAngle.getDegrees() > -90) {
        //         ramseteCommand = new RamseteCommandWrapper(drivetrain, start, new ArrayList<Translation2d>(), endPoint, AutoConstants.RAMSETE_CONSTANTS);
        //     }

            
        // }

        if (distance > ideal + acceptableError) {
            
            ramseteCommand = new RamseteCommandWrapper(drivetrain, start, new ArrayList<Translation2d>(), endPoint, AutoConstants.RAMSETE_CONSTANTS);
            ramseteCommand.schedule();
        }
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        if (ramseteCommand != null) {
            if (ramseteCommand.isScheduled()) {
                ramseteCommand.cancel();
            }
        }
        ramseteCommand = null;
    }
}