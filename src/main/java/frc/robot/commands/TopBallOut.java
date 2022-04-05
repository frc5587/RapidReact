package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;

/** Ejects the top cargo out of the shooter */
public class TopBallOut extends CommandBase {
    private final Kicker rightKicker, leftKicker;
    private final Shooter shooter;

    public TopBallOut(Kicker rightKicker, Kicker leftKicker, Shooter shooter) {
        this.rightKicker = rightKicker;
        this.leftKicker = leftKicker;
        this.shooter = shooter;

        addRequirements(rightKicker, leftKicker, shooter);
    }

    /*
    Enable kicker PID
    */
    @Override
    public void initialize() {
        rightKicker.enable();
        leftKicker.enable();

        shooter.enable();
        shooter.setVelocity(10);
    }

    @Override
    public void execute() {
        if (shooter.atSetpoint()) {
            rightKicker.moveDistance(1);
            leftKicker.moveDistance(1);

        }
    }

    /* When the command ends, turn off the conveyor */
    @Override
    public void end(boolean interruptible) {
        rightKicker.disable();
        leftKicker.disable();
        shooter.disable();
    }
} 