package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberArmConstants;

public class ClimberArm extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(ClimberArmConstants.MOTOR, MotorType.kBrushless);

    public ClimberArm() {
        configureSparkMax();
    }

    private void configureSparkMax() {
        motor.restoreFactoryDefaults();

        motor.setInverted(ClimberArmConstants.MOTOR_INVERTED);

        motor.setSmartCurrentLimit(ClimberArmConstants.STALL_LIMIT, ClimberArmConstants.FREE_LIMIT);

        motor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Move motor forwards
     */
    public void forwards() {
        motor.set(0.3);
    }

    /**
     * Move motor backwards
     */
    public void backwards() {
        motor.set(-0.3);
    }

    /**
     * Stop motor
     */
    public void stop() {
        motor.set(0);
    }


}
