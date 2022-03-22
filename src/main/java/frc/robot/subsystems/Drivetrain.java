package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.frc5587.lib.subsystems.DrivetrainBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends DrivetrainBase {
    private static final WPI_TalonFX leftLeader =new WPI_TalonFX(DrivetrainConstants.LEFT_LEADER);
    private static final WPI_TalonFX leftFollower = new WPI_TalonFX(DrivetrainConstants.LEFT_FOLLOWER);
    private static final WPI_TalonFX rightLeader = new WPI_TalonFX(DrivetrainConstants.RIGHT_LEADER);
    private static final WPI_TalonFX rightFollower= new WPI_TalonFX(DrivetrainConstants.RIGHT_FOLLOWER);
    private Field2d field = new Field2d();
    private Rotation2d lastRotation = new Rotation2d();
    private double angularVelocity = 0;

    private static DriveConstants driveConstants = new DriveConstants(
        DrivetrainConstants.WHEEL_DIAMETER,
        DrivetrainConstants.HISTORY_LIMIT, 
        DrivetrainConstants.INVERT_GYRO,
        DrivetrainConstants.ENCODER_EPR, 
        DrivetrainConstants.GEARING,
        AutoConstants.TRACK_WIDTH
    );

    public Drivetrain() {
        // this(new WPI_TalonFX(DrivetrainConstants.LEFT_LEADER), new WPI_TalonFX(DrivetrainConstants.LEFT_FOLLOWER),
        //         new WPI_TalonFX(DrivetrainConstants.RIGHT_LEADER), new WPI_TalonFX(DrivetrainConstants.RIGHT_FOLLOWER));
        super(new MotorControllerGroup(leftLeader, leftFollower), new MotorControllerGroup(rightLeader, rightFollower), 
        driveConstants);
        zeroOdometry();
        SmartDashboard.putData(field);
    }

    private ChassisSpeeds getChassisSpeeds() {
        return AutoConstants.DRIVETRAIN_KINEMATICS.toChassisSpeeds(new DifferentialDriveWheelSpeeds());
    }

    public double getAngularVelocity() {
        return angularVelocity;
    }

    public double getLinearVelocity() {
        return getChassisSpeeds().vxMetersPerSecond;
    }

    // public Drivetrain(WPI_TalonFX leftLeader, WPI_TalonFX leftFollower, WPI_TalonFX rightLeader, WPI_TalonFX rightFollower) {
    //     super(new MotorControllerGroup(leftLeader, leftFollower), new MotorControllerGroup(rightLeader, rightFollower), 
    //     driveConstants);

    //     // System.out.println("!! " + leftLeader + "  " + );

    //     this.leftLeader = leftLeader;
    //     this.leftFollower = leftFollower;
    //     this.rightLeader = rightLeader;
    //     this.rightFollower = rightFollower;
    // }

    @Override
    public void configureMotors() {
        leftLeader.configFactoryDefault();
        rightLeader.configFactoryDefault();
        leftFollower.configFactoryDefault();
        rightFollower.configFactoryDefault();

        leftLeader.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake);
        leftFollower.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);

        leftLeader.setInverted(DrivetrainConstants.LEFT_SIDE_INVERTED);
        rightLeader.setInverted(DrivetrainConstants.RIGHT_SIDE_INVERTED);
        leftFollower.setInverted(DrivetrainConstants.LEFT_SIDE_INVERTED);
        rightFollower.setInverted(DrivetrainConstants.RIGHT_SIDE_INVERTED);

        leftLeader.configSupplyCurrentLimit(DrivetrainConstants.SUPPLY_CURRENT_LIMIT_CONFIGURATION);
        rightLeader.configSupplyCurrentLimit(DrivetrainConstants.SUPPLY_CURRENT_LIMIT_CONFIGURATION);
        leftFollower.configSupplyCurrentLimit(DrivetrainConstants.SUPPLY_CURRENT_LIMIT_CONFIGURATION);
        rightFollower.configSupplyCurrentLimit(DrivetrainConstants.SUPPLY_CURRENT_LIMIT_CONFIGURATION);

        resetEncoders();
    }

    @Override
    protected double getLeftPositionTicks() {
        return leftLeader.getSelectedSensorPosition() * (DrivetrainConstants.LEFT_ENCODERS_INVERTED ? -1:1);
    }

    @Override
    protected double getRightPositionTicks() {
        return rightLeader.getSelectedSensorPosition() * (DrivetrainConstants.RIGHT_ENCODERS_INVERTED ? -1:1);
    }

    @Override
    protected double getLeftVelocityTicksPerSecond() {
        return (leftLeader.getSelectedSensorVelocity() * (DrivetrainConstants.LEFT_ENCODERS_INVERTED ? -1:1) / DrivetrainConstants.VELOCITY_DENOMINATOR);
    }

    @Override
    protected double getRightVelocityTicksPerSecond() {
        return (rightLeader.getSelectedSensorVelocity() * (DrivetrainConstants.RIGHT_ENCODERS_INVERTED ? -1:1) / DrivetrainConstants.VELOCITY_DENOMINATOR);
    }

    @Override
    protected void resetEncoders() {
        leftLeader.setSelectedSensorPosition(0);
        rightLeader.setSelectedSensorPosition(0);
    }

    @Override
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        super.tankDriveVolts(-leftVolts, -rightVolts);
    }

    @Override
    public void periodic() {
        super.periodic();

        angularVelocity = (getRotation2d().getRadians() - lastRotation.getRadians()) / .02;

        // System.out.println(angularVelocity);

        field.setRobotPose(getPose().getX(), getPose().getY(), getRotation2d());
        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
        SmartDashboard.putNumber("Angle", getHeading());
        SmartDashboard.putNumber("LeftEncoder", getLeftPositionTicks());
        SmartDashboard.putNumber("RightEncoder", getRightPositionTicks());
        SmartDashboard.putNumber("LeftMeters", getLeftPositionMeters());
        SmartDashboard.putNumber("RightMeters", getRightPositionMeters());
    }
}