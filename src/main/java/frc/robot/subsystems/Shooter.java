package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;

public class Shooter extends SubsystemBase {
    private static final double error_threshold = 0.01;

    private final WPI_TalonFX leaderMotor = new WPI_TalonFX(ShooterConstants.SHOOTER_LEADER_MOTOR);
    private final WPI_TalonFX followerMotor = new WPI_TalonFX(ShooterConstants.SHOOTER_FOLLOWER_MOTOR);
    private final MotorControllerGroup shooterMotors = new MotorControllerGroup(leaderMotor, followerMotor);
    private boolean enabled = false;
    private double setpoint = 0;
    private final double shotEfficiency = 0.8; // how efficient the flywheel is a transferring momentum
    private final double flywheelCargoVelocityRatio = 2; // ratio between the velocity of the flywheel and release velocity of cargo

    public Shooter() {
        configureShooterFalcon();

        SmartDashboard.setDefaultNumber("Velocity", 0);
    }

    public void configureShooterFalcon() {
        resetEncoders();
        leaderMotor.configFactoryDefault();
        followerMotor.configFactoryDefault();

        leaderMotor.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIGURATION);
        followerMotor.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIGURATION);

        leaderMotor.setInverted(ShooterConstants.SHOOTER_LEADER_INVERTED);
        followerMotor.setInverted(ShooterConstants.SHOOTER_FOLLOWER_INVERTED);

        leaderMotor.setNeutralMode(NeutralMode.Brake);
        followerMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setVelocity(double velocity) {
        setpoint = velocity;
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        stop();
        enabled = false;
        stopVoltage();
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void stopVoltage() {
        shooterMotors.setVoltage(0);
    }
    
    public void stop() {
        setVelocity(0);
    }

    public void resetEncoders() {
        leaderMotor.setSelectedSensorPosition(0);
        followerMotor.setSelectedSensorPosition(0);
    }

    public double getVelocity() {
        // Encoder velocity divided by (EPR * (wheel circumference / gearing)
        return (leaderMotor.getSelectedSensorVelocity() /
                (ShooterConstants.ENCODER_EPR * ShooterConstants.VELOCITY_DENOMINATOR) *
                (2 * Math.PI) * (ShooterConstants.WHEEL_RADIUS / ShooterConstants.GEARING));
    }

    public double getSmartDashboard() {
        return SmartDashboard.getNumber("Velocity", 0);
    }

    public boolean atSetpoint() {
        return Math.abs(setpoint - getVelocity()) / setpoint < error_threshold && setpoint != 0;
    }

    public double shooterRegression(double distance) {
        return ((0.248525 * Math.pow(distance, 2)) + Math.pow(0.0156791, ((-1.00889 * distance) + 4.25483)) + 15.3616);
    }

    /**
     * ! this is highly approximate
     */
    public double horizontalCargoVelocity(double distance) {
        return (shotEfficiency * shooterRegression(distance) / flywheelCargoVelocityRatio * Math.cos(ShooterConstants.SHOOTER_ANGLE));
    }

    /**
     * ! this is highly approximate
     */
    public double horizontalCargoVelocityToShooterVelocity(double horizontalVelocity) {
        return flywheelCargoVelocityRatio * horizontalVelocity / (Math.cos(ShooterConstants.SHOOTER_ANGLE) * shotEfficiency);
    }

    /**
     * ! this is highly approximate
     */
    public double timeOfFlight(double distance) {
        return distance / horizontalCargoVelocity(distance);
    }

    public double shootDistanceStationary(double distance) {
        if (Units.metersToInches(distance) > 180 || Units.metersToInches(distance) < 100) {
            SmartDashboard.putBoolean("In Range", false);
            return 0.0;
        } else {
            SmartDashboard.putBoolean("In Range", true);
            return shooterRegression(distance);
        }
    }

    public double shootDistanceMoving(double velocity, double movingAngle, double distance) {
        final double effect = 1;
        return (horizontalCargoVelocityToShooterVelocity(velocity * Math.cos(movingAngle) * effect) + shootDistanceStationary(distance));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Setpoint", setpoint);
        SmartDashboard.putNumber("Shooter Velocity", getVelocity());
        SmartDashboard.putBoolean("Shooter At Threshold?", atSetpoint());

        if (isEnabled()) {
            shooterMotors.setVoltage(ShooterConstants.SHOOTER_FF.calculate(setpoint)
                    - ShooterConstants.PID.calculate(setpoint - getVelocity()));
        }
    }
}
