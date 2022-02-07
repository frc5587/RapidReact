package frc.robot.subsystems;

import org.frc5587.lib.subsystems.SimpleMotorBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SimpleMotorBase {
    private static CANSparkMax motor = new CANSparkMax(25, MotorType.kBrushless);

    public Intake() {
        super(motor);
        configureMotors();
    }

    public void configureMotors() {
        motor.restoreFactoryDefaults();

        motor.setInverted(false);

        motor.setIdleMode(IdleMode.kCoast);
    }
}
