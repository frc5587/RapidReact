package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants.TurretConstants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class Turret extends ProfiledPIDSubsystem {
    private static CANSparkMax turretMotor = new CANSparkMax(TurretConstants.TURRET_MOTOR, MotorType.kBrushless);
    private static RelativeEncoder encoder = turretMotor.getEncoder();

    public Turret() {
        super(new ProfiledPIDController(
            Constants.TurretConstants.PID.kP,
            Constants.TurretConstants.PID.kI,
            Constants.TurretConstants.PID.kD,
            Constants.TurretConstants.CONSTRAINTS
        ));
        configureTurretSpark();
    }

    public void configureTurretSpark() {
        turretMotor.restoreFactoryDefaults();

        turretMotor.setInverted(TurretConstants.MOTOR_INVERTED);

        turretMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setTurret(double turretSpeed) {
        turretMotor.set(-turretSpeed);
    }

    public void moveWithThrottle(double throttle) {
        turretMotor.set(throttle);
    }

    public void stopTurret() {
        turretMotor.set(0);
    }

    // public void setMotorThrottle(double throttle) {
    //     turretMotor.set(throttle);
    //     System.out.println("Yes we are the moving at this speed " + throttle);
    // }

    public void resetEncoders() {
        encoder.setPosition(0);
    }

    protected double getPositionDegrees() {
        return (encoder.getPosition() / Constants.TurretConstants.GEARING / Constants.TurretConstants.ENCODER_CPR);
    }

    protected double getPositionRadians() {
        return Math.toRadians(getPositionDegrees());
    }

    @Override
    protected double getMeasurement() {
        return getPositionRadians();
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        moveWithThrottle(output);
    }
}