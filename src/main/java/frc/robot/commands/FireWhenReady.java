package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Conveyor.ControlMode;

/** Pushes balls into the shooter once it is at the desired spin-up speed */
public class FireWhenReady extends CommandBase {
    private final Conveyor conveyor;
    private final Turret turret;
    private final Kicker leftKicker, rightKicker;
    private final Shooter shooter;
    private final Limelight limelight;

    public FireWhenReady(Conveyor conveyor, Kicker rightKicker, Kicker leftKicker, Shooter shooter, Limelight limelight, Turret turret) {
        this.conveyor = conveyor;
        this.rightKicker = rightKicker;
        this.leftKicker = leftKicker;
        this.shooter = shooter;
        this.limelight = limelight;
        this.turret = turret;

        addRequirements(rightKicker, leftKicker);
    }

    @Override
    public void initialize() {
        conveyor.setShooterControlled(true);
        
        rightKicker.enable();
        leftKicker.enable();

        if (!conveyor.isIndexRunning()) {
            conveyor.setControlMode(ControlMode.VELOCITY);
            conveyor.setVelocity(0);
        }
    }

    @Override
    public void execute() {
        if (shooter.atSetpoint() && shooter.isInRange(limelight.calculateDistance()) && limelight.hasTarget() && turret.inProperPosition()) {
            rightKicker.moveDistance(1);
            leftKicker.moveDistance(1);
            conveyor.setVelocity(3);
        }
    }

    @Override
    public void end(boolean interrupted) {

        if (!conveyor.isIndexRunning()) {
            conveyor.setControlMode(ControlMode.OFF);
        }

        conveyor.setShooterControlled(false);
    }
}