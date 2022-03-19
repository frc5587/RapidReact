package frc.robot.subsystems;

import frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private static final double error_threshold = 0.01;

    private final WPI_TalonFX leaderMotor = new WPI_TalonFX(ShooterConstants.SHOOTER_LEADER_MOTOR);
    private final WPI_TalonFX followerMotor = new WPI_TalonFX(ShooterConstants.SHOOTER_FOLLOWER_MOTOR);
    private final MotorControllerGroup shooterMotors = new MotorControllerGroup(leaderMotor, followerMotor);
    private boolean enabled = false;
    private double setpoint = 0;

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
        return enabled == true;
    }
    
    public void stop() {
        setVelocity(0);
    }

    public void stopVoltage() {
        shooterMotors.setVoltage(0);
    }

    public void resetEncoders() {
        leaderMotor.setSelectedSensorPosition(0);
        followerMotor.setSelectedSensorPosition(0);
    }

    public double getVelocity() {
        return (leaderMotor.getSelectedSensorVelocity() / (ShooterConstants.ENCODER_EPR * ShooterConstants.VELOCITY_DENOMINATOR) * (2 * Math.PI) * (ShooterConstants.WHEEL_RADIUS / ShooterConstants.GEARING));
    }

    public double getSmartDashboard() {
        return SmartDashboard.getNumber("Velocity", 0);
    }

    public boolean atSetpoint() {
        return Math.abs(setpoint - getVelocity()) / setpoint < error_threshold && setpoint != 0;
    }

    public double shootDistanceStationary(double distance) {
        return ((0.248525 * Math.pow(distance, 2)) + Math.pow(0.0156791, ((-1.00889 * distance) + 4.25483)) + 15.3616);
    }

    /**
     * Calculate the flywheel velocity to spin up to, while factoring in drivetrain velocity.
     * <p>
     * @see {@link https://www.desmos.com/calculator/yad5bwclbs}
     * @param drivetrain
     * @param turret
     * @param limelight
     * @param distance - distance to shoot
     * @return double velocity of the flywheel
     */
    public double shootDistanceMoving(Drivetrain drivetrain, Turret turret, Limelight limelight, double distance) {
        return (((drivetrain.getLinearVelocity() * Math.cos(turret.getPositionRadians() - limelight.getHorizontalAngle())) / (0.7 * Math.cos(Units.degreesToRadians(ShooterConstants.SHOOTER_ANGLE))) * 2) + shootDistanceStationary(distance));
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Setpoint", setpoint);
        SmartDashboard.putNumber("PID Output", ShooterConstants.PID.calculate(setpoint - getVelocity()));
        SmartDashboard.putNumber("Voltage", ShooterConstants.SHOOTER_FF.calculate(setpoint) - ShooterConstants.PID.calculate(setpoint - getVelocity()));
        SmartDashboard.putNumber("Real Velocity", getVelocity());
        SmartDashboard.putNumber("Feedforward", ShooterConstants.SHOOTER_FF.calculate(setpoint));
        SmartDashboard.putBoolean("At threshold", atSetpoint());

        if(isEnabled()) {
            shooterMotors.setVoltage(ShooterConstants.SHOOTER_FF.calculate(setpoint) - ShooterConstants.PID.calculate(setpoint - getVelocity()));
        }
        
        // if(getVelocity() != 0)
            // System.out.println(getVelocity() + "  " + getSmartDashboard() + "  "+ setpoint + "  " + ShooterConstants.PID.calculate(setpoint - getVelocity()));
    }
}
