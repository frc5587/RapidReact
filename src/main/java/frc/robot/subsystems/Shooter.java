package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
    private static final double error_threshold = 0.01;

    private final WPI_TalonFX leaderMotor = new WPI_TalonFX(ShooterConstants.SHOOTER_LEADER_MOTOR);
    private final WPI_TalonFX followerMotor = new WPI_TalonFX(ShooterConstants.SHOOTER_FOLLOWER_MOTOR);
    private final MotorControllerGroup shooterMotors = new MotorControllerGroup(leaderMotor, followerMotor);
    private boolean enabled = false;
    private double setpoint = 0;
    private final double shotEfficiency = 0.8; // how efficient the flywheel is a transferring momentum
    private final double flywheelCargoVelocityRatio = 2; // ratio between the velocity of the flywheel and release velocity of cargo
    private final Limelight limelight;

    public Shooter(Limelight limelight) {
        configureShooterFalcon();

        this.limelight = limelight;
    }

    public void configureShooterFalcon() {
        resetEncoders();
        leaderMotor.configFactoryDefault();
        followerMotor.configFactoryDefault();

        leaderMotor.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIGURATION);
        followerMotor.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIGURATION);

        leaderMotor.setInverted(ShooterConstants.SHOOTER_LEADER_INVERTED);
        followerMotor.setInverted(ShooterConstants.SHOOTER_FOLLOWER_INVERTED);

        leaderMotor.setNeutralMode(NeutralMode.Coast);
        followerMotor.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Sets the setpoint to a desired velocity
     * @param velocity the desired velocity in m/s of the wheel surface
     */
    public void setVelocity(double velocity) {
        setpoint = velocity;
    }

    /**
     * enable PID
     */
    public void enable() {
        enabled = true;
    }

    /** Sets velocity setpoint to 0, sets motor output to 0, and disables PID */
    public void disable() {
        stop();
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void stopVoltage() {
        setVelocity(0);
        shooterMotors.setVoltage(0);
    }
    
    public void stop() {
        setVelocity(0);
        shooterMotors.set(0);
    }

    public void resetEncoders() {
        leaderMotor.setSelectedSensorPosition(0);
        followerMotor.setSelectedSensorPosition(0);
    }

    /**
     * @return shooter flywheel velocity in m/s of the wheel surface
     */
    public double getVelocity() {
        // Encoder velocity divided by (EPR * (wheel circumference / gearing))
        return (leaderMotor.getSelectedSensorVelocity() /
                (ShooterConstants.ENCODER_EPR * ShooterConstants.VELOCITY_DENOMINATOR) *
                (2 * Math.PI) * (ShooterConstants.WHEEL_RADIUS / ShooterConstants.GEARING));
    }

    /**
     * Returns true if the shooter's actual velocity is within a certain error of the setpoint
     */
    public boolean atSetpoint() {
        return Math.abs(setpoint - getVelocity()) / setpoint < error_threshold && setpoint != 0;
    }

    /**
     * Calculates a shooter velocity based on a given target distance in meters.
     * The equation can be found here: https://www.desmos.com/calculator/sy0vwazxzz
     * @param distance the distance from the target in meters
     * @return a shooter velocity after calculation
     */
    public double shooterRegression(double distance) {
        return ((0.248525 * Math.pow(distance, 2)) + Math.pow(0.0156791, ((-1.00889 * distance) + 4.25483)) + 15.3616);
    }

    /* ! this is highly approximate */
    public double horizontalCargoVelocity(double distance) {
        return (shotEfficiency * shooterRegression(distance) / flywheelCargoVelocityRatio * Math.cos(ShooterConstants.SHOOTER_ANGLE));
    }

    /* ! this is highly approximate */
    public double horizontalCargoVelocityToShooterVelocity(double horizontalVelocity) {
        return flywheelCargoVelocityRatio * horizontalVelocity / (Math.cos(ShooterConstants.SHOOTER_ANGLE) * shotEfficiency);
    }

    /* ! this is highly approximate */
    public double timeOfFlight(double distance) {
        return distance / horizontalCargoVelocity(distance);
    }

    /**
     * Returns a calculated value from shooterRegression() if the distance is within our
     * minimum and maximum safe shoot distances.
     * @param distance the distance from the target
     * @return  a calculated velocity from shooterRegression if the target is
     *          within min/max shoot range, or 0 if the target is not within range
     */
    public double shootDistanceStationary(double distance) {
        return shooterRegression(distance);
    }

    public boolean isInRange(double distance) {
        return (distance < ShooterConstants.MAX_SHOOT_DISTANCE && distance > ShooterConstants.MIN_SHOOT_DISTANCE);
    }

    //TODO: check this documentation!!!
    /**
     * Calculates the shooter velocity accounting for the movement of the robot
     * @param velocity the velocity of the robot
     * @param movingAngle the angular movement of the robot in radians
     * @param distance the distance from the target
     * @return a velocity based on shooterRegression, adjusted for robot movement
     */
    public double shootDistanceMoving(double velocity, double movingAngle, double distance) {
        final double effect = -1;
        return (horizontalCargoVelocityToShooterVelocity(velocity * Math.cos(movingAngle) * effect) 
                + shootDistanceStationary(distance));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Setpoint", setpoint);
        SmartDashboard.putNumber("Shooter Velocity", getVelocity());
        SmartDashboard.putBoolean("Shooter At Setpoint?", atSetpoint());

        if (isEnabled()) {
            shooterMotors.setVoltage(ShooterConstants.SHOOTER_FF.calculate(setpoint)
                    - ShooterConstants.PID.calculate(setpoint - getVelocity()));
        }

        SmartDashboard.putBoolean("In Range", isInRange(limelight.calculateDistance()));
    }
}
