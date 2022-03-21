package frc.robot.subsystems;

import frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends ProfiledPIDSubsystem {
    private final CANSparkMax turretMotor = new CANSparkMax(TurretConstants.TURRET_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder encoder = turretMotor.getEncoder();
    private State lastSetpoint = new State(0, 0);
    private final double limit = TurretConstants.LIMIT;

    public Turret() {
        super(TurretConstants.PID);
        configureTurretSpark();
    }

    public void configureTurretSpark() {
        resetEncoders();
        turretMotor.restoreFactoryDefaults();
        turretMotor.setInverted(TurretConstants.TURRET_MOTOR_INVERTED);
        turretMotor.setIdleMode(IdleMode.kBrake);
        turretMotor.setSmartCurrentLimit(TurretConstants.STALL_CURRENT_LIMIT, TurretConstants.FREE_CURRENT_LIMIT);
    }

    // uses radians per second
    public void setThrottle(double throttle) {
        turretMotor.set(throttle);
    }

    public void setPosition(double position) {
        if(Math.abs(position) >= limit) {
            if ((Math.abs(getPositionRadians())) > (Math.abs(position))) {
                setGoal(position);
            }
            System.out.println(position + " is not allowed.");
        } else {
            setGoal(position);
        }
    }

    public void setVelocityAtPosition(double position, double velocity) {
        if(Math.abs(position) >= limit) {
            System.out.println(position + " is not allowed.");
        } else {
            setGoal(new State(position, velocity));
        }
    }

    public void stopTurret() {
        turretMotor.set(0);
    }

    public void resetEncoders() {
        encoder.setPosition(0);
    }

    public double getPositionRadians() {
        return (encoder.getPosition() * (2 * Math.PI) / TurretConstants.GEARING);
    }

    public double getVelocity() {
        return (encoder.getVelocity() / 60 * (2 * Math.PI) / TurretConstants.GEARING);
    }

    @Override
    protected double getMeasurement() {
        return getPositionRadians();
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        turretMotor.setVoltage(TurretConstants.TURRET_FF.calculate(setpoint.velocity, (setpoint.velocity - lastSetpoint.velocity)/.02) + output);
        lastSetpoint = setpoint;
    }

    public boolean isFinished() {
        return lastSetpoint.velocity == 0;
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Turret Position", getPositionRadians());
    }
}