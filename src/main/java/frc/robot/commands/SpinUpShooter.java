package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;
import frc.robot.Constants.ShooterConstants;

/** Spins up the shooter to the desired velocity and does not end */
public class SpinUpShooter extends CommandBase {
    private final Shooter shooter;
    private final Drivetrain drivetrain;
    private final Limelight limelight;
    private final double distanceOffset = 0.1;
    private final NetworkTableEntry speedEntry = SmartDashboard.getEntry("Shooter test speed");

    public SpinUpShooter(Shooter shooter, Drivetrain drivetrain, Limelight limelight) {
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.limelight = limelight;

        addRequirements(shooter);

        if (!speedEntry.exists()) {
            speedEntry.setDouble(0);
        }
    }

    @Override
    public void initialize() {
        shooter.enable();
    }

    @Override
    public void execute() {
        double distance = limelight.getDistanceToHub();
        double offAngle = limelight.getRelativeAngleToHub().getRadians();

        double speed = MathUtil.clamp(shooter.shootDistanceMoving(drivetrain.getLinearVelocity(), offAngle, distance - distanceOffset), shooter.shootDistanceStationary(ShooterConstants.MIN_SHOOT_DISTANCE), shooter.shootDistanceStationary(ShooterConstants.MAX_SHOOT_DISTANCE));

        shooter.setVelocity(speedEntry.getDouble(0));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.disable();
    }
}