package frc.robot.subsystems;

import org.frc5587.lib.subsystems.LimelightBase;
import frc.robot.Constants.ShooterConstants;

public class Limelight extends LimelightBase {
    public Limelight() {
        super(ShooterConstants.MOUNT_ANGLE, ShooterConstants.LENS_HEIGHT, ShooterConstants.GOAL_HEIGHT_METERS);
    }
}
