package frc.robot.commands;

import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
// TODO DEBUGGING CODE
public class ShootDashboard extends CommandBase {
    private Shooter shooter;
    private Kicker rightKicker;
    private Kicker leftKicker;
    private DoubleSupplier throttleSupplier;

    public ShootDashboard(Shooter shooter, Kicker rightKicker, Kicker leftKicker, DoubleSupplier throttleSupplier) {
        this.shooter = shooter;
        this.rightKicker = rightKicker;
        this.leftKicker = leftKicker;
        this.throttleSupplier = throttleSupplier;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        rightKicker.enable();
        leftKicker.enable();
        rightKicker.moveMore(1);
        leftKicker.moveMore(1);

        shooter.enable();
    }

    @Override
    public void execute() {
        // Set the velocity of the shooter to the throttle speed set in SmartDashboard ("Velocity")
        shooter.setVelocity(throttleSupplier.getAsDouble());
    }

    /*
    When the command ends, stop the shooter
    */
    @Override
    public void end(boolean interrupted) {
        shooter.disable();
    }
}
