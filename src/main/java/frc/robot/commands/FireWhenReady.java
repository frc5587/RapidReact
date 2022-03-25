package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Conveyor.ControlMode;

/** Pushes balls into the shooter once it is at the desired spin-up speed */
public class FireWhenReady extends CommandBase {
    private final Conveyor conveyor;
    private final Kicker leftKicker, rightKicker;
    private final Shooter shooter;

    public FireWhenReady(Conveyor conveyor, Kicker rightKicker, Kicker leftKicker, Shooter shooter) {
        this.conveyor = conveyor;
        this.rightKicker = rightKicker;
        this.leftKicker = leftKicker;
        this.shooter = shooter;

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
        System.out.println("shooter firewhenready!!!");
        if (shooter.atSetpoint()) {
            rightKicker.moveDistance(1);
            leftKicker.moveDistance(1);

            conveyor.setVelocity(1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("firewhenready ending!!! " + interrupted);
        conveyor.setControlMode(ControlMode.OFF);
        rightKicker.disable();
        leftKicker.disable();
        shooter.disable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}