package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class ProfileTurret extends CommandBase {
    private final Turret turret;
    private final DoubleSupplier throttleSupplier;
    
    public ProfileTurret(Turret turret, DoubleSupplier throttleSupplier) {
        this.turret = turret;
        this.throttleSupplier = throttleSupplier;

        addRequirements(turret);
    }

    double lastVelocity = 0;
    @Override
    public void execute() {
        turret.disable();
        double throttle = throttleSupplier.getAsDouble();
        turret.setThrottle(throttle * -.1);
        lastVelocity = turret.getVelocity();
    }

    @Override
    public void end(boolean interrupted) {
        turret.setThrottle(0);
        turret.resetEncoders();
    }
}
