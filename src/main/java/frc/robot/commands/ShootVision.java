package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Conveyor.ControlMode;;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootVision extends CommandBase {
    private Shooter shooter;
    private Limelight limelight;
    private Conveyor conveyor;
    private Kicker leftKicker, rightKicker;
    private LinebreakSensor linebreakSensor;
    private boolean linebroken = false;

    public ShootVision(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;

        addRequirements(shooter, limelight);
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
            conveyor.setVelocity(2);
            // if (!linebreakSensor.isCrossed()) {
            //     rightKicker.moveMore(1);
            //     leftKicker.moveMore(1);

            //     conveyor.setControlMode(ControlMode.VELOCITY);
            //     conveyor.setVelocity(2);
            // } else {
            //     if (!linebroken) {
            //         conveyor.setControlMode(ControlMode.POSITION);
            //         conveyor.moveMore(0.5);
            //         rightKicker.moveMore(.4);
            //         leftKicker.moveMore(.4);
            //     }

            //     linebroken = true;

            // }
        }
        // While the ball is still in the robot, move it slowly to pop it out of the robot.
        // if (!linebreakSensor.isCrossed() && !linebroken) {
        //     rightKicker.moveMore(1);
        //     leftKicker.moveMore(1);
        // } else {
        //     if(!linebroken) {
        //         conveyor.setControlMode(ControlMode.POSITION);
        //         conveyor.moveMore(0.5);
        //         rightKicker.moveMore(.3);
        //         leftKicker.moveMore(.3);
        //     }
        //     linebroken = true;
        // }
        // rightKicker.setGoal(rightKicker.getPosition());
        // leftKicker.setGoal(leftKicker.getPosition());
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