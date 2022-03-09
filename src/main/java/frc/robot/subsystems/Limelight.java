package frc.robot.subsystems;

import org.frc5587.lib.subsystems.LimelightBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

public class Limelight extends LimelightBase {
    public Limelight() {
        super(ShooterConstants.MOUNT_ANGLE, ShooterConstants.LENS_HEIGHT, ShooterConstants.GOAL_HEIGHT_METERS);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Calculated distance", calculateDistance() + 0.7);
    }
}
