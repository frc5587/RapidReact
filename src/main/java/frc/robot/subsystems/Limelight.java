package frc.robot.subsystems;

import org.frc5587.lib.subsystems.LimelightBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

public class Limelight extends LimelightBase {
    public Limelight() {
        super(ShooterConstants.MOUNT_ANGLE, ShooterConstants.LENS_HEIGHT, ShooterConstants.GOAL_HEIGHT_METERS, ShooterConstants.DISTANCE_OFFSET);
    }

    @Override
    public void periodic() {
        // System.out.println(calculateDistance());
        SmartDashboard.putNumber("Calculated distance", calculateDistance());
    }
}
