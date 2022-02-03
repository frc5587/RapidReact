package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MoveSpark extends SubsystemBase {
    protected CANSparkMax motor = new CANSparkMax(25, MotorType.kBrushless);

    public MoveSpark() {
        configureMotors();
    }

    public void configureMotors() {
        motor.restoreFactoryDefaults();

        motor.setInverted(false);

        motor.setIdleMode(IdleMode.kCoast);
    }

    public void moveByThrottle(double throttle) {
        motor.set(throttle);
    }

    public void stop() {
        motor.set(0);
    }
}
