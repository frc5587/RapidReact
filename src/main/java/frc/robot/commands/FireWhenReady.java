package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Conveyor.ControlMode;

/** Pushes balls into the shooter once it is at the desired spin-up speed */
public class FireWhenReady extends CommandBase {
    private final Conveyor conveyor;
    private final Kicker leftKicker, rightKicker;
    private final Shooter shooter;
    private final Limelight limelight;

    public FireWhenReady(Conveyor conveyor, Kicker rightKicker, Kicker leftKicker, Shooter shooter, Limelight limelight) {
        this.conveyor = conveyor;
        this.rightKicker = rightKicker;
        this.leftKicker = leftKicker;
        this.shooter = shooter;
        this.limelight = limelight;

        addRequirements(conveyor, rightKicker, leftKicker);
    }

    @Override
    public void initialize() {
        rightKicker.enable();
        leftKicker.enable();
        conveyor.setControlMode(ControlMode.VELOCITY);
        conveyor.setVelocity(0);
    }

    @Override
    public void execute() {
        if (shooter.atSetpoint() && shooter.isInRange(limelight.calculateDistance()) && limelight.hasTarget()) {
            rightKicker.moveDistance(1);
            leftKicker.moveDistance(1);
            conveyor.setVelocity(1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setControlMode(ControlMode.OFF);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}