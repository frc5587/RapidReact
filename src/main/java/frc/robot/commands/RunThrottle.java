package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MoveSpark;

public class RunThrottle extends CommandBase{
    public DoubleSupplier throttleSupplier;
    public MoveSpark spark;

    public RunThrottle(MoveSpark spark, DoubleSupplier throttleSupplier) {
        this.throttleSupplier = throttleSupplier;
        this.spark = spark;

        addRequirements(spark);
    }

    @Override
    public void execute() {
        spark.moveByThrottle(throttleSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        spark.stop();
    }
}
