package frc.robot.subsystems;

import frc.robot.Constants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Extend the intake system to allow for ball intake & outtake
 */
public class ClimbPistons extends SubsystemBase {
    public DoubleSolenoid pistonSet1 = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            ClimbPistonsConstants.SET_1_FORWARD_CHANNEL, ClimbPistonsConstants.SET_1_REVERSE_CHANNEL);
    public DoubleSolenoid pistonSet2 = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            ClimbPistonsConstants.SET_2_FORWARD_CHANNEL, ClimbPistonsConstants.SET_2_REVERSE_CHANNEL);

    public ClimbPistons() {
    }

    /**
     * There are two sets of pistons, with two pistons in each set. This code was
     * written with the idea that there would be 4 motors in total - 2 for the inner
     * climb and 2 for the outer climb. Therefore, the pistonSet1 could be 2 pistons
     * for the inner climb, and the pistonSet2 could be the other 2 pistons for the
     * outer climb.
     */
    public void extendSet1() {
        pistonSet1.set(DoubleSolenoid.Value.kForward);
    }

    public void extendSet2() {
        pistonSet2.set(DoubleSolenoid.Value.kForward);
    }

    public void retractSet1() {
        pistonSet1.set(DoubleSolenoid.Value.kReverse);
    }

    public void retractSet2() {
        pistonSet2.set(DoubleSolenoid.Value.kReverse);
    }
}
