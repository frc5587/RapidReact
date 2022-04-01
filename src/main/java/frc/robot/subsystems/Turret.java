package frc.robot.subsystems;

import frc.robot.Constants.TurretConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Turret extends PIDSubsystem {
    private final CANSparkMax turretMotor = new CANSparkMax(TurretConstants.TURRET_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder turretEncoder = turretMotor.getEncoder();
    private final Rotation2d upperLimit = new Rotation2d(TurretConstants.LIMIT);
    private final Rotation2d lowerLimit = new Rotation2d(-TurretConstants.LIMIT);
    private Rotation2d targetPosition = new Rotation2d();
    private Rotation2d targetVelocity = new Rotation2d();

    public Turret() {
        super(TurretConstants.PID);

        configureMotors();
    }

    public void configureMotors() {
        resetEncoders();
        turretMotor.restoreFactoryDefaults();
        turretMotor.setInverted(TurretConstants.TURRET_MOTOR_INVERTED);
        turretMotor.setIdleMode(IdleMode.kBrake);
        turretMotor.setSmartCurrentLimit(TurretConstants.STALL_CURRENT_LIMIT, TurretConstants.FREE_CURRENT_LIMIT);
    }

    public void setThrottle(double throttle) {
        turretMotor.set(throttle);
    }

    public boolean isMoveAllowed(Rotation2d position) {
        return isInBounds(position) || ((Math.abs(getPosition().getRadians())) > (Math.abs(position.getRadians())));
    }

    public boolean isInBounds(Rotation2d position) {
        return (position.getRadians() < upperLimit.getRadians() && position.getRadians() > lowerLimit.getRadians());
    }

    public boolean isTurretInBounds() {
        return isInBounds(getPosition());
    }

    public boolean isMoveInSameDirectionFromHome(Rotation2d position, Rotation2d velocity) {
        return (velocity.getRadians() > 0 && position.getRadians() > 0) || (velocity.getRadians() < 0 && position.getRadians() < 0);
    }

    public void setPose(Rotation2d position) {
        setPose(position, new Rotation2d());
    }

    public void setPose(Rotation2d position, Rotation2d velocity) {
        if (isMoveAllowed(position)) {
            setSetpoint(position.getRadians());

            targetPosition = position;
            targetVelocity = velocity;
        }
    }

    public void setVelocity(Rotation2d velocity) {
        setPose(getPosition(), velocity);
    }

    public void stopTurret() {
        turretMotor.set(0);
    }

    public void resetEncoders() {
        turretEncoder.setPosition(0);
    }

    public Rotation2d getPosition() {
        return new Rotation2d(turretEncoder.getPosition() * (2 * Math.PI) / TurretConstants.GEARING);
    }

    public Rotation2d getVelocity() {
        return new Rotation2d(turretEncoder.getVelocity() / 60 * (2 * Math.PI) / TurretConstants.GEARING);
    }

    @Override
    protected double getMeasurement() {
        return getPosition().getRadians();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        if (!isTurretInBounds() && isMoveInSameDirectionFromHome(targetPosition, targetVelocity)) {
            turretMotor.set(output);

        } else {
            turretMotor.set(TurretConstants.TURRET_FF.calculate(targetVelocity.getRadians()) + output);
        }

    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Turret Pos Degrees", getPosition().getDegrees());
    }
}