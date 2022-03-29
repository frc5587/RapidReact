package frc.robot.subsystems;

import frc.robot.Constants.TurretConstants;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Turret extends ProfiledPIDSubsystem {
    private final CANSparkMax turretMotor = new CANSparkMax(TurretConstants.TURRET_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder turretEncoder = turretMotor.getEncoder();
    private final double upperLimit = TurretConstants.LIMIT;
    private final double lowerLimit = -TurretConstants.LIMIT;
    private State lastSetpoint = new State(0, 0);
    private final DoubleSupplier velocityCompensationSupplier;
    private boolean isCompensatingVelocity = false;

    public Turret(DoubleSupplier velocityCompensationSupplier) {
        super(TurretConstants.PID);

        this.velocityCompensationSupplier = velocityCompensationSupplier;

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

    /**
     * Sets the position of the turret, unless the position is outside of the
     * turret's soft limits.
     * @param position the desired position in radians
     */
    public void setPosition(double position) {
        System.out.println("TURRET SET!!!!!!!!!!!");
        if(position >= upperLimit || position <= lowerLimit) {
            if ((Math.abs(getPositionRadians())) > (Math.abs(position))) {
                setGoal(position);
            } else {
                System.out.println(position + " is not allowed.");
            }
        } else {
            setGoal(position);
        }
    }

    /**
     * Sets the position of the turret accounting for a given velocity offset
     * @param position the desired position in radians
     * @param velocity the velocity in radians per second
     */
    public void setVelocityAtPosition(double position, double velocity) {
        if((position >= upperLimit || position <= lowerLimit) && ((Math.abs(getPositionRadians())) < (Math.abs(position)))) {
            System.out.println(position + " is not allowed.");
        } else {
            setGoal(new State(position, velocity));
        }
    }

    public void enableVelocityCompensation() {
        isCompensatingVelocity = true;
    }

    public void disableVelocityCompensation() {
        isCompensatingVelocity = false;
    }

    public void stopTurret() {
        turretMotor.set(0);
    }

    public void resetEncoders() {
        turretEncoder.setPosition(0);
    }

    public double getPositionRadians() {
        return (turretEncoder.getPosition() * (2 * Math.PI) / TurretConstants.GEARING);
    }

    public double getVelocity() {
        return (turretEncoder.getVelocity() / 60 * (2 * Math.PI) / TurretConstants.GEARING);
    }

    @Override
    protected double getMeasurement() {
        return getPositionRadians();
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        double acceleration = (setpoint.velocity - lastSetpoint.velocity)/.02;
        double compensation = isCompensatingVelocity ? velocityCompensationSupplier.getAsDouble() : 0;

        turretMotor.setVoltage(TurretConstants.TURRET_FF.calculate(setpoint.velocity - compensation, acceleration) + output);
        lastSetpoint = setpoint;
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Turret Pos Degrees", Units.radiansToDegrees(getPositionRadians()));
    }

    public boolean isFinished() {
        return lastSetpoint.velocity == 0;
    }
}