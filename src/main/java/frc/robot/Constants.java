// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.auto.RamseteCommandWrapper.RamseteConstants;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Pose2d;
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
    public static final class DrivetrainConstants {
        /* motor ids */
        public static final int LEFT_LEADER = 10;
        public static final int LEFT_FOLLOWER = 11;
        public static final int RIGHT_LEADER = 15;
        public static final int RIGHT_FOLLOWER = 16;

        /* motor inversions */
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

        public static final double WHEEL_DIAMETER = Units.inchesToMeters(6.3065);
        public static final int HISTORY_LIMIT = 32;
        public static final double ENCODER_EPR = 2048;
        public static final double GEARING = (54. / 20.) * (50. / 12.);
        public static final boolean INVERT_GYRO = false;
        public static final double VELOCITY_DENOMINATOR = 0.1;
        public static final double VOLTAGE_RAMP_RATE = 0.4;

        // TODO likely in between 0.25 & 0.5. Test these to find the right value
        public static final double QUICKTURN_CURVE_MULTIPLIER = 0.5;
    }

    public static final class AutoConstants {
        public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(
                0.66725, 2.3304, 0.3317);
        public static final PIDController PID_CONTROLLER = new PIDController(3.2003, 0, 0);

        public static final double TRACK_WIDTH = 0.7;

        public static final double THEORETICAL_TOP_SPEED = (12 - FEEDFORWARD.ks) / FEEDFORWARD.kv;

        public static final double MAXIMUM_VELOCITY = 2; // m/s
        public static final double MAXIMUM_ACCELERATION = 1; // m/s^2
        public static final double CENTRIPETAL_ACCELERATION = 0.2; // m/s^2

        public static final DifferentialDriveKinematics DRIVETRAIN_KINEMATICS = new DifferentialDriveKinematics(
                TRACK_WIDTH);

        public static final RamseteConstants RAMSETE_CONSTANTS = new RamseteConstants(
                FEEDFORWARD, PID_CONTROLLER, MAXIMUM_VELOCITY, MAXIMUM_ACCELERATION, 
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

        /* motor inversions */
        public static final boolean RIGHT_HOOK_MOTOR_INVERTED = true;
        public static final boolean LEFT_HOOK_MOTOR_INVERTED = false;
        public static final boolean RIGHT_STICK_ARM_MOTOR_INVERTED = true;
        public static final boolean LEFT_STICK_ARM_MOTOR_INVERTED = false;

        /* motor current limits */
        public static final int STALL_CURRENT_LIMIT = 40;
        public static final int FREE_CURRENT_LIMIT = 35;

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
                0, 0, 0, CONSTRAINTS);
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
        public static final int FORWARD_CHANNEL = 0;
        public static final int REVERSE_CHANNEL = 3;
    }

    /*
     * INTAKE
     */
    public static class IntakeConstants {
        /* motor ids */
        public static final int INTAKE_MOTOR = 20;

        /* motor inversions */
        public static final boolean MOTOR_INVERTED = false;

        /* motor current limits */
        public static final int STALL_CURRENT_LIMIT = 35;
        public static final int FREE_CURRENT_LIMIT = 30;
        public static final double SMART_CURRENT_LIMIT_DELAY = 0.2; // in seconds

        /* pid/ff values */
        public static final PIDController PID = new PIDController(1.5841, 0, 0);
        public static final SimpleMotorFeedforward INTAKE_FF = new SimpleMotorFeedforward(0.10904, 3.9107, 0.081738);

        public static final double GEARING = 10;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(1);

        public static final double DRIVETRAIN_VELOCITY_OFFSET = 1;
        public static final double MIN_VELOCITY = 4;
    }

    /*
     * CONVEYOR
     */
    public static class ConveyorConstants {
        /* motor ids */
        public static final int CONVEYOR_MOTOR = 30;

        /* motor inversions */
        public static final boolean MOTOR_INVERTED = false;

        /* motor current limits */
        public static final int STALL_CURRENT_LIMIT = 35;
        public static final int FREE_CURRENT_LIMIT = 30;

        /* pid/ff values */
        // TODO: Check out high constraint
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

        /* motor inversions */
        public static final boolean RIGHT_KICKER_INVERTED = true;
        public static final boolean LEFT_KICKER_INVERTED = false;

        /* motor current limits */
        public static final int STALL_CURRENT_LIMIT = 35;
        public static final int FREE_CURRENT_LIMIT = 30;

        /* pid/ff values */
        // TODO Should be the same as conveyor, check constraints
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

        /* motor inversions */
        public static final boolean TURRET_MOTOR_INVERTED = false;

        /* motor current limits */
        public static final int STALL_CURRENT_LIMIT = 35;
        public static final int FREE_CURRENT_LIMIT = 30;

        /* pid/ff values */
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(6, 6);
        public static final PIDController PID = new PIDController(
                9.65, 0, 1.0649);
        public static final SimpleMotorFeedforward TURRET_FF = new SimpleMotorFeedforward(0.27429, 0.86838, 0.049705);

        public static final int GEARING = 90;
        public static final double THROTTLE_MULTIPLIER = 0.1;
        public static final double LIMIT = Units.degreesToRadians(90);

        public static final Translation2d HUB_POSITION = new Translation2d(Units.inchesToMeters(324), Units.inchesToMeters(162));
    }

    /*
     * SHOOTER
     */
    public static final class ShooterConstants {
        /* motor ids */
        public static final int SHOOTER_LEADER_MOTOR = 40;
        public static final int SHOOTER_FOLLOWER_MOTOR = 41;

        /* motor inversions */
        public static final boolean SHOOTER_LEADER_INVERTED = false;
        public static final boolean SHOOTER_FOLLOWER_INVERTED = true;

        /* motor current limits */
        public static final int SMART_CURRENT_LIMIT = 35;
        public static final int HARD_CURRENT_LIMIT = 40;
        public static final double SMART_CURRENT_LIMIT_DELAY = 0.2; // seconds
        public static final SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT_CONFIGURATION = new SupplyCurrentLimitConfiguration(
                true,
                DrivetrainConstants.SMART_CURRENT_LIMIT,
                DrivetrainConstants.HARD_CURRENT_LIMIT,
                DrivetrainConstants.SMART_CURRENT_LIMIT_DELAY);

        /* pid/ff values */
        public static final PIDController PID = new PIDController(.8, 0, 0);
        public static final SimpleMotorFeedforward SHOOTER_FF = new SimpleMotorFeedforward(0.64689, 0.33756, 0.029811);

        public static final double GEARING = 1;
        public static final double ENCODER_EPR = 2048;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
        public static final double VELOCITY_DENOMINATOR = 0.1;
        public static final double SHOOTER_ANGLE = 70;
        public static final double MIN_SHOOT_DISTANCE = Units.inchesToMeters(100);
        public static final double MAX_SHOOT_DISTANCE = Units.inchesToMeters(180);
        public static final double DEFAULT_SPIN_UP_VELOCITY = 17.6;
    }

    /*
     * LIMELIGHT CONSTANTS
     */
    public static final class LimelightConstants {
        public static final double GOAL_HEIGHT_METERS = Units.inchesToMeters(103); // is 2.578 meters;
        public static final double LENS_HEIGHT_METERS = Units.inchesToMeters(32.007); // is 0.813 meters
        public static final double MOUNT_ANGLE = 30; // in degrees
        public static final double DISTANCE_OFFSET = 0.677;
    }
}