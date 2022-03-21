package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LockTurret extends CommandBase {
    private final Turret turret;
    private final Limelight limelight;
    private final ClimbThrottle climbCommand;

    public LockTurret(Turret turret, Limelight limelight, ClimbThrottle climbCommand) {
        this.turret = turret;
        this.limelight = limelight;
        this.climbCommand = climbCommand;

        addRequirements(turret, limelight);
    }

    @Override
    public void initialize() {
        turret.enable();
    }

    @Override
    public void execute() {
        if(limelight.hasTarget() && !climbCommand.isClimbing) {
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