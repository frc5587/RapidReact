// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.auto.ConstrainedTrajectory.TrajectoryConstraints;
import org.frc5587.lib.auto.RamseteCommandWrapper.RamseteConstants;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /*
     * DRIVETRAIN
     */
    public static final class DrivetrainConstants {
        /* motor ids */
        public static final int LEFT_LEADER = 10;
        public static final int LEFT_FOLLOWER = 11;
        public static final int RIGHT_LEADER = 15;
        public static final int RIGHT_FOLLOWER = 16;

        /* inversions */
        public static final boolean LEFT_MOTORS_INVERTED = true;
        public static final boolean RIGHT_MOTORS_INVERTED = false;
        
        public static final boolean LEFT_ENCODERS_INVERTED = true;
        public static final boolean RIGHT_ENCODERS_INVERTED = true;

        /* motor current limits */
        public static final int SMART_CURRENT_LIMIT = 35;
        public static final int HARD_CURRENT_LIMIT = 40;
        public static final double SMART_CURRENT_LIMIT_DELAY = 0.2; // in seconds
        public static final SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT_CONFIGURATION = new SupplyCurrentLimitConfiguration(
                true,
                DrivetrainConstants.SMART_CURRENT_LIMIT,
                DrivetrainConstants.HARD_CURRENT_LIMIT,
                DrivetrainConstants.SMART_CURRENT_LIMIT_DELAY);

        /* DriveConstants values */
        public static final double WHEEL_DIAMETER = .505 / Math.PI;
        public static final int HISTORY_LIMIT = 32;
        public static final double ENCODER_EPR = 2048;
        public static final double GEARING = (54. / 20.) * (50. / 12.);
        public static final boolean INVERT_GYRO = false;
        public static final double VELOCITY_DENOMINATOR = 0.2;
        public static final double VOLTAGE_RAMP_RATE = 0.4;

        public static final double QUICKTURN_CURVE_MULTIPLIER = 0.75;
    }

    /*
     * AUTO
     */
    public static final class AutoConstants {
        /* pid/ff values */
        public static final SimpleMotorFeedforward DRIVETRAIN_FF = new SimpleMotorFeedforward(
                0.66725, 2.3304, 0.3317);
        public static final PIDController PID_CONTROLLER = new PIDController(3.2003, 0, 0);

        /* kinematics */
        public static final double TRACK_WIDTH = 0.7;

        public static final DifferentialDriveKinematics DRIVETRAIN_KINEMATICS = new DifferentialDriveKinematics(
            TRACK_WIDTH);

        /* Ramsete values */
        public static final double MAXIMUM_VELOCITY = 2; // m/s
        public static final double MAXIMUM_ACCELERATION = 1; // m/s^2
        public static final double CENTRIPETAL_ACCELERATION = 1; // m/s^2

        /* Ramsete */
        public static final RamseteConstants RAMSETE_CONSTANTS = new RamseteConstants(
            DRIVETRAIN_FF, PID_CONTROLLER, MAXIMUM_VELOCITY, MAXIMUM_ACCELERATION, 
                CENTRIPETAL_ACCELERATION, DRIVETRAIN_KINEMATICS);
        
        /* trajectory */
        public static final TrajectoryConstraints TRAJECTORY_CONSTRAINTS = new TrajectoryConstraints(
            DRIVETRAIN_FF, MAXIMUM_VELOCITY, MAXIMUM_ACCELERATION, 
                CENTRIPETAL_ACCELERATION, DRIVETRAIN_KINEMATICS);
    }

    /*
     * CLIMB
     */
    public static final class ClimbConstants {
        /* motor ids */
        public static final int RIGHT_HOOK_ARM_MOTOR = 50;
        public static final int LEFT_HOOK_ARM_MOTOR = 51;
        public static final int RIGHT_STICK_ARM_MOTOR = 55;
        public static final int LEFT_STICK_ARM_MOTOR = 56;

        /* inversions */
        public static final boolean RIGHT_HOOK_MOTOR_INVERTED = true;
        public static final boolean LEFT_HOOK_MOTOR_INVERTED = false;
        public static final boolean RIGHT_STICK_ARM_MOTOR_INVERTED = false;
        public static final boolean LEFT_STICK_ARM_MOTOR_INVERTED = true;

        /* motor current limits */
        public static final int STALL_CURRENT_LIMIT = 45;
        public static final int FREE_CURRENT_LIMIT = 45;

        /* pid/ff values */
        // TODO Characterize climb & check TrapezoidProfile constraints
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(0.3, 1);

        public static final ProfiledPIDController HOOK_UNLOADED_PID = new ProfiledPIDController(
                0, 0, 0, CONSTRAINTS);
        public static final ProfiledPIDController HOOK_LOADED_PID = new ProfiledPIDController(
                0, 0, 0, CONSTRAINTS);
        public static final ElevatorFeedforward UNLOADED_HOOK_FF = new ElevatorFeedforward(0, 0, 0, 0);
        public static final ElevatorFeedforward LOADED_HOOK_FF = new ElevatorFeedforward(0, 0, 0, 0);

        public static final ProfiledPIDController STICK_UNLOADED_PID = new ProfiledPIDController(
                60, 0, 0, CONSTRAINTS);
        public static final ProfiledPIDController STICK_LOADED_PID = new ProfiledPIDController(
                0, 0, 0, CONSTRAINTS);
        public static final ElevatorFeedforward UNLOADED_STICK_FF = new ElevatorFeedforward(0, 0, 0);
        public static final ElevatorFeedforward LOADED_STICK_FF = new ElevatorFeedforward(0, 0, 0);

        public static final double SPOOL_RADIUS = 0.02;
        public static final double HOOK_GEARING = 16;
        public static final double STICK_GEARING = 15;
    }

    /*
     * INTAKE PISTONS
     */
    public static final class IntakePistonsConstants {
        /* solenoid channels */
        public static final int FORWARD_CHANNEL = 2;
        public static final int REVERSE_CHANNEL = 0;
    }

    /*
     * INTAKE
     */
    public static class IntakeConstants {
        /* motor ids */
        public static final int INTAKE_MOTOR = 20;

        /* inversions */
        public static final boolean MOTOR_INVERTED = true;

        /* motor current limits */
        public static final int STALL_CURRENT_LIMIT = 35;
        public static final int FREE_CURRENT_LIMIT = 30;

        /* pid/ff values */
        public static final PIDController PID = new PIDController(1.5841, 0, 0);
        public static final SimpleMotorFeedforward INTAKE_FF = new SimpleMotorFeedforward(0.10904, 3.9107, 0.081738);

        public static final double GEARING = 10;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(1);

        public static final double DRIVETRAIN_VELOCITY_OFFSET = 2; // Intake will move at the Drivetrain's velocity multiplied by this value
        public static final double MIN_VELOCITY = 4;
    }

    /*
     * CONVEYOR
     */
    public static class ConveyorConstants {
        /* motor ids */
        public static final int CONVEYOR_MOTOR = 30;

        /* inversions */
        public static final boolean MOTOR_INVERTED = true;

        /* motor current limits */
        public static final int STALL_CURRENT_LIMIT = 35;
        public static final int FREE_CURRENT_LIMIT = 30;

        /* pid/ff values */
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(2, 20);

        public static final ProfiledPIDController POSITION_PID = new ProfiledPIDController(
                4.3574, 0, 0.12733, CONSTRAINTS);
        public static final ProfiledPIDController VELOCITY_PID = new ProfiledPIDController(
                0.17574, 0, 0, CONSTRAINTS);
        public static final SimpleMotorFeedforward CONVEYOR_FF = new SimpleMotorFeedforward(
                0.078977, 1.9659, 0.057779);

        public static final int GEARING = 10;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
    }

    /*
     * KICKER
     */
    public static class KickerConstants {
        /* motor ids */
        public static final int RIGHT_KICKER_MOTOR = 35;
        public static final int LEFT_KICKER_MOTOR = 36;

        /* inversions */
        public static final boolean RIGHT_KICKER_INVERTED = true;
        public static final boolean LEFT_KICKER_INVERTED = false;

        /* motor current limits */
        public static final int STALL_CURRENT_LIMIT = 35;
        public static final int FREE_CURRENT_LIMIT = 30;

        /* pid/ff values */
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(2, 10);

        public static final ProfiledPIDController RIGHT_KICKER_PID = new ProfiledPIDController(
                4.4195, 0, 0, CONSTRAINTS);
        public static final ProfiledPIDController LEFT_KICKER_PID = new ProfiledPIDController(
                3.5583, 0, 0, CONSTRAINTS);
        public static final SimpleMotorFeedforward RIGHT_KICKER_FF = new SimpleMotorFeedforward(0.15312, 1.9677,
                0.27331);
        public static final SimpleMotorFeedforward LEFT_KICKER_FF = new SimpleMotorFeedforward(0.067363, 1.9044,
                0.14625);

        public static final int GEARING = 10;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
    }

    /*
     * TURRET
     */
    public static final class TurretConstants {
        /* motor ids */
        public static final int TURRET_MOTOR = 45;

        /* inversions */
        public static final boolean TURRET_MOTOR_INVERTED = false;

        /* motor current limits */
        public static final int STALL_CURRENT_LIMIT = 35;
        public static final int FREE_CURRENT_LIMIT = 30;

        /* pid/ff values */
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(6, 6);

        public static final PIDController PID = new PIDController(
                2, 0, 0.03);
        public static final SimpleMotorFeedforward TURRET_FF = new SimpleMotorFeedforward(0.2756, 0.8555, 0.05829);

        public static final int GEARING = 90;
        public static final double THROTTLE_MULTIPLIER = 0.1;
        public static final double LIMIT = Units.degreesToRadians(100); // Turret will not move past this position in either direction
    }

    /*
     * SHOOTER
     */
    public static final class ShooterConstants {
        /* motor ids */
        public static final int SHOOTER_LEADER_MOTOR = 40;
        public static final int SHOOTER_FOLLOWER_MOTOR = 41;

        /* inversions */
        public static final boolean SHOOTER_LEADER_INVERTED = false;
        public static final boolean SHOOTER_FOLLOWER_INVERTED = true;

        /* motor current limits */
        public static final int SMART_CURRENT_LIMIT = 35;
        public static final int HARD_CURRENT_LIMIT = 40;
        public static final double SMART_CURRENT_LIMIT_DELAY = 0.2; // in seconds
        public static final SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT_CONFIGURATION = new SupplyCurrentLimitConfiguration(
                true,
                DrivetrainConstants.SMART_CURRENT_LIMIT,
                DrivetrainConstants.HARD_CURRENT_LIMIT,
                DrivetrainConstants.SMART_CURRENT_LIMIT_DELAY);

        /* pid/ff values */
        public static final PIDController PID = new PIDController(1, 0, 0);
        public static final SimpleMotorFeedforward SHOOTER_FF = new SimpleMotorFeedforward(0.76075, 0.35258, 0.03031);

        public static final double GEARING = 1;
        public static final double ENCODER_EPR = 2048;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
        public static final double VELOCITY_DENOMINATOR = 0.1;

        public static final double SHOOTER_ANGLE = 64;
        public static final double MIN_SHOOT_DISTANCE = 1.9; // in meters
        public static final double MAX_SHOOT_DISTANCE = 8.5; // in meters
    }

    /*
     * LIMELIGHT CONSTANTS
     */
    public static final class LimelightConstants {
        public static final double GOAL_HEIGHT_METERS = Units.inchesToMeters(101.5);
        public static final double LENS_HEIGHT_METERS = Units.inchesToMeters(33);
        public static final double MOUNT_ANGLE = 30;
        public static final double DISTANCE_OFFSET = 0.677;

        public static final Translation2d HUB_POSITION = new Translation2d(Units.inchesToMeters(324), Units.inchesToMeters(162));
    }
}