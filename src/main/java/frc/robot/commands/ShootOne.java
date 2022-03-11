package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Conveyor.ControlMode;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootOne extends CommandBase {
    private final Conveyor conveyor;
    private final Kicker rightKicker, leftKicker;
    private final LinebreakSensor linebreakSensor;
    private final Shooter shooter;
    private final DoubleSupplier velocitySupplier;
    private boolean linebroken = false;
    @Deprecated()
    public ShootOne(Conveyor conveyor, Kicker rightKicker, Kicker leftKicker, LinebreakSensor linebreakSensor, Shooter shooter, DoubleSupplier velocitySupplier) {
        this.conveyor = conveyor;
        this.rightKicker = rightKicker;
        this.leftKicker = leftKicker;
        this.linebreakSensor = linebreakSensor;
        this.shooter = shooter;
        this.velocitySupplier = velocitySupplier;

        addRequirements(conveyor, rightKicker, leftKicker, linebreakSensor, shooter);
    }

    /*
    Enable kicker PID
    */
    @Override
    public void initialize() {
        rightKicker.enable();
        leftKicker.enable();

        shooter.enable();
        shooter.setVelocity(velocitySupplier.getAsDouble());

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

    // @Override
    // public boolean isFinished() {
    //     return !linebreakSensor.isCrossed() && linebroken;
    // }
} 