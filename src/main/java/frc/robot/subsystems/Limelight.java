package frc.robot.subsystems;

import org.frc5587.lib.subsystems.LimelightBase;

import frc.robot.Constants.LimelightConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends LimelightBase {
    public Limelight() {
        super(LimelightConstants.MOUNT_ANGLE, LimelightConstants.LENS_HEIGHT_METERS, 
                LimelightConstants.GOAL_HEIGHT_METERS, LimelightConstants.DISTANCE_OFFSET);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Limelight Distance", calculateDistance());
        SmartDashboard.putBoolean("Has Target", hasTarget());
    }
}
