package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Turret extends SubsystemBase {
    private final CANSparkMax turretMotor = new CANSparkMax(TurretConstants.TURRET_MOTOR, MotorType.kBrushless);

    public Turret() {
        configureTurretSpark();
    }

    public void configureTurretSpark() {
        turretMotor.restoreFactoryDefaults();

        turretMotor.setSoftLimit(SoftLimitDirection.kForward, 3);
        turretMotor.setSoftLimit(SoftLimitDirection.kReverse, -3);

        turretMotor.setIdleMode(IdleMode.kCoast);
    }

    public void setTurret(double turretSpeed) {
        turretMotor.setIdleMode(IdleMode.kCoast);
        turretMotor.set(-turretSpeed);
    }

    public void stopTurret() {
        turretMotor.set(0);
        turretMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setMotorThrottle(double throttle) {
        turretMotor.set(throttle);
    }
}