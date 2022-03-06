package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootDashboard extends CommandBase {
    private Shooter shooter;
    private double throttleSupplier;

    public ShootDashboard(Shooter shooter, double throttleSupplier) {
        this.shooter = shooter;
        this.throttleSupplier = throttleSupplier;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        // Set the velocity of the shooter to the throttle speed set in SmartDashboard ("Velocity")
        shooter.setVelocity(-throttleSupplier);
    }

    /*
    When the command ends, stop the shooter
    */
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shooter.resetEncoders();
    }
}
