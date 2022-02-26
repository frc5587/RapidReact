package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class RunKickerUp extends CommandBase {
    private final Conveyor conveyor;
    private final Kicker rightKicker;
    private final Kicker leftKicker;


    public RunKickerUp(Conveyor conveyor, Kicker rightKicker, Kicker leftKicker) {
        this.conveyor = conveyor;
        this.rightKicker = rightKicker;
        this.leftKicker = leftKicker;
        addRequirements(conveyor, rightKicker, leftKicker);
    }

    @Override
    public void initialize() {
        conveyor.enable();
        rightKicker.enable();
        leftKicker.enable();

        conveyor.resetEncoders();
        rightKicker.resetEncoders();
        leftKicker.resetEncoders();
        
        conveyor.setDistance(Units.inchesToMeters(16));
        rightKicker.setDistance(Units.inchesToMeters(8));
        leftKicker.setDistance(Units.inchesToMeters(8));
    }

    // @Override
    // public boolean isFinished() {
    //     System.out.println(!rightKicker.isSpinning() && !leftKicker.isSpinning());
    //     return (!rightKicker.isSpinning() && !leftKicker.isSpinning());
    // }
}

