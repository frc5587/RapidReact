package frc.robot.commands;

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
        turret.enableVelocityCompensation();
    }

    @Override
    public void execute() {
        if(limelight.hasTarget()) {
            double error = -limelight.getHorizontalAngle(); 
            double distance = limelight.calculateDistance();
            double sideBallTravel = shooter.timeOfFlight(distance) * drivetrain.getLinearVelocity() * Math.sin(turret.getPositionRadians() + error);
            double angleAdjustment = Math.atan2(sideBallTravel, distance);
            double totalError = error + angleAdjustment;
            if (Math.abs(totalError) > 0.06) { //TODO: try changing this number to make turret motion better
                turret.setVelocityAtPosition(turret.getPositionRadians() + totalError, -drivetrain.getAngularVelocity());
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.disableVelocityCompensation();
        turret.setPosition(0);
    }
}