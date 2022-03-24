package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants.TurretConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class Turret extends ProfiledPIDSubsystem {
    private final CANSparkMax turretMotor = new CANSparkMax(TurretConstants.TURRET_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder turretEncoder = turretMotor.getEncoder();
    private final double limit = TurretConstants.LIMIT;
    private State lastSetpoint = new State(0, 0);

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

    public void setPosition(double position) {
        if(Math.abs(position) >= limit) {
            if ((Math.abs(getPositionRadians())) > (Math.abs(position))) {
                setGoal(position);
            } else {
                System.out.println(position + " is not allowed.");
            }
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