package frc.robot.subsystems;

import frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final WPI_TalonFX leaderMotor = new WPI_TalonFX(ShooterConstants.SHOOTER_LEADER_MOTOR);
    private final WPI_TalonFX followerMotor = new WPI_TalonFX(ShooterConstants.SHOOTER_FOLLOWER_MOTOR);

    private final MotorControllerGroup shooterMotors = new MotorControllerGroup(leaderMotor, followerMotor);

    private double velocitySetpoint = 0;

    public Shooter() {
        configureShooterFalcon();
    }

    public void configureShooterFalcon() {
        leaderMotor.configFactoryDefault();
        followerMotor.configFactoryDefault();

        leaderMotor.setInverted(ShooterConstants.SHOOTER_LEADER_INVERTED);
        followerMotor.setInverted(ShooterConstants.SHOOTER_FOLLOWER_INVERTED);

        leaderMotor.setNeutralMode(NeutralMode.Brake);
        followerMotor.setNeutralMode(NeutralMode.Brake);

        resetEncoders();
    }

    public void setVelocity(double velocity) {
        velocitySetpoint = velocity;
    }

    public void setThrottle(double throttle) {
        shooterMotors.set(throttle);
    }

    public void stop() {
        shooterMotors.set(0);
        velocitySetpoint = 0;
    }

    public void resetEncoders() {
        leaderMotor.setSelectedSensorPosition(0);
        followerMotor.setSelectedSensorPosition(0);
    }

    public double getVelocity() {
        return (leaderMotor.getSelectedSensorVelocity() / (ShooterConstants.ENCODER_EPR * ShooterConstants.VELOCITY_DENOMINATOR) * (2 * Math.PI) * (ShooterConstants.WHEEL_RADIUS / ShooterConstants.GEARING));
    }

    @Override
    public void periodic() {
        super.periodic();

        shooterMotors.setVoltage(ShooterConstants.SHOOTER_FF.calculate(velocitySetpoint) + ShooterConstants.PID.calculate(velocitySetpoint - getVelocity()));
    }
}
