package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LockTurret extends CommandBase {
    private final Turret turret;
    private final Limelight limelight;

    public LockTurret(Turret turret, Limelight limelight, Drivetrain drivetrain) {
        this.turret = turret;
        this.limelight = limelight;

        addRequirements(turret, limelight);
    }

    @Override
    public void initialize() {
        turret.enable();
    }

    @Override
    public void execute() {
        // TODO CHECK IF WE'RE CLIMBING AND DONT RUN TURRET TRACK
        if(limelight.hasTarget()) {
            double error = limelight.getHorizontalAngle();             
            if (Math.abs(error) > 0.03) {
                turret.setPosition(turret.getPositionRadians() - error);
            }
        }
        else {
            turret.setPosition(0);
        }
    }
}