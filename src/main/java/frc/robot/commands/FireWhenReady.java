package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Conveyor.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

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

        /*
    Enable kicker PID
    */
    @Override
    public void initialize() {
        rightKicker.enable();
        leftKicker.enable();
    }

    @Override
    public void execute() {
        if (shooter.atSetpoint()) {
            rightKicker.moveMore(1);
            leftKicker.moveMore(1);

            conveyor.setControlMode(ControlMode.VELOCITY);
            conveyor.setVelocity(1);
        }
    }

    /*
    When the command ends, turn off the conveyor
    */
    @Override
    public void end(boolean interruptable) {
        conveyor.setControlMode(ControlMode.OFF);
        shooter.disable();
        rightKicker.disable();
        leftKicker.disable();
    }
}