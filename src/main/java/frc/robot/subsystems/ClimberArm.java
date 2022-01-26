package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberArmConstants;

public class ClimberArm extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(ClimberArmConstants.MOTOR, MotorType.kBrushless);
    private final DoubleSolenoid hand = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    public ClimberArm() {
        configureSparkMax();
    }

    private void configureSparkMax() {
        motor.restoreFactoryDefaults();

        motor.setInverted(ClimberArmConstants.MOTOR_INVERTED);

        // motor.setSmartCurrentLimit(ClimberArmConstants.STALL_LIMIT, ClimberArmConstants.FREE_LIMIT);

        motor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Move motor forwards
     */
    public void forwards() {
        System.out.println("fwd");
        motor.set(ClimberArmConstants.FORWARD_THROTTLE);
    }
    
    /**
     * Move motor backwards
     */
    public void backwards() {
        System.out.println("bkwd");
        motor.set(-ClimberArmConstants.BACKWARDS_THROTTLE);
    }

    /**
     * Stop motor
     */
    public void stop() {
        System.out.println("stop");
        motor.set(0);
    }

    public void grip() {
        hand.set(Value.kForward);
    }

    public void release() {
        hand.set(Value.kReverse);
    }
}
