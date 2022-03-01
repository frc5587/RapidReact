package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BlockMotor extends CommandBase {
    private final Drivetrain drivetrain;

    private static Timer timer = new Timer();
    private int throttle = 1;

    public BlockMotor(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setThrottle(-1);
        timer.start();
    }

    @Override
    public void execute() {
        drivetrain.setThrottle(throttle);
        if(timer.hasElapsed(30)) {
            throttle *= -1;
            timer.reset();
        }
    }
}
