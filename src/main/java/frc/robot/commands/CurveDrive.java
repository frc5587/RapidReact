package frc.robot.commands;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class CurveDrive extends CommandBase {
    private final Drivetrain drivetrain;
    private final DoubleSupplier throttleSupplier, curveSupplier;
    private final BooleanSupplier quickTurnSupplier;

    public CurveDrive(Drivetrain drivetrain, DoubleSupplier throttleSupplier, DoubleSupplier curveSupplier, BooleanSupplier quickTurnSupplier) {
        this.drivetrain = drivetrain;
        this.throttleSupplier = throttleSupplier;
        this.curveSupplier = curveSupplier;
        this.quickTurnSupplier = quickTurnSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double throttle = throttleSupplier.getAsDouble();
        double curve = curveSupplier.getAsDouble();
        boolean quickTurn = quickTurnSupplier.getAsBoolean();

        curve *= quickTurn? DrivetrainConstants.QUICKTURN_CURVE_MULTIPLIER : 1;

        drivetrain.curvatureDrive(throttle, curve, quickTurn);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}