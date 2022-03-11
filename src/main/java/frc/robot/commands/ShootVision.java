package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Conveyor.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootVision extends CommandBase {
    private Shooter shooter;
    private Limelight limelight;
    private Conveyor conveyor;
    private Kicker leftKicker, rightKicker;
    private LinebreakSensor linebreakSensor;
    private boolean linebroken = false;

    public ShootVision(Conveyor conveyor, Kicker rightKicker, Kicker leftKicker, LinebreakSensor linebreakSensor, Shooter shooter, Limelight limelight) {
        this.conveyor = conveyor;
        this.rightKicker = rightKicker;
        this.leftKicker = leftKicker;
        this.linebreakSensor = linebreakSensor;
        this.shooter = shooter;
        this.limelight = limelight;

        addRequirements(conveyor, rightKicker, leftKicker, linebreakSensor, shooter, limelight);
    }

        /*
    Enable kicker PID
    */
    @Override
    public void initialize() {
        rightKicker.enable();
        leftKicker.enable();

        shooter.enable();
        shooter.setVelocity(shooter.shootDistance(limelight.calculateDistance()));

        linebroken = false;
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
        rightKicker.disable();
        leftKicker.disable();
        shooter.disable();
    }
}