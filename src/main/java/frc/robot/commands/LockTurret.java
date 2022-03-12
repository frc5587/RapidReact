package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LockTurret extends CommandBase {
    private Turret turret;
    private Limelight limelight;
    private Drivetrain drivetrain;

    public LockTurret(Turret turret, Limelight limelight, Drivetrain drivetrain) {
        this.turret = turret;
        this.limelight = limelight;
        this.drivetrain = drivetrain;

        addRequirements(turret, limelight);
    }

    @Override
    public void initialize() {
        turret.enable();
    }

    @Override
    public void execute() {
        // System.out.println("Execute LockTurret");
        if(limelight.hasTarget()) {
            double error = limelight.getHorizontalAngle();
            // System.out.println(turret.getPositionRadians() - error + "  " + turret.getPositionRadians() + "  " + limelight.getHorizontalAngle());
             
            if (Math.abs(error) > 0.03) {
                turret.setPosition(turret.getPositionRadians() - error);
            }
        }
    }
}