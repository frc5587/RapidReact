package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;

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

    @Override
    public void execute() {
        if(limelight.hasTarget()) {
            double error = -limelight.getHorizontalAngle(); 
            double distance = limelight.calculateDistance();
            double sideBallTravel = shooter.timeOfFlight(distance) * drivetrain.getLinearVelocity() * Math.sin(turret.getPositionRadians() + error);
            double angleAdjustment = Math.atan2(sideBallTravel, distance);
            double totalError = error + angleAdjustment;
            if (Math.abs(totalError) > 0.03) {
                turret.setVelocityAtPosition(turret.getPositionRadians() + totalError, -drivetrain.angularVelocity());
            }
        }
        else {
            turret.setPosition(0);
        }
    }
}