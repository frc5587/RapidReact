// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import org.frc5587.lib.auto.RamseteCommandWrapper.RamseteConstants;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.controller.PIDController;
import org.frc5587.lib.pid.PID;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
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
        public static final int LEFT_LEADER = 10;
        public static final int LEFT_FOLLOWER = 11;
        public static final int RIGHT_LEADER = 15;
        public static final int RIGHT_FOLLOWER = 16;

        public static final boolean LEFT_SIDE_INVERTED = true;
        public static final boolean RIGHT_SIDE_INVERTED = false;
        public static final boolean LEFT_ENCODERS_INVERTED = true;
        public static final boolean RIGHT_ENCODERS_INVERTED = true;

        // motor current limits
        public static final int SMART_CURRENT_LIMIT = 35;
        public static final int HARD_CURRENT_LIMIT = 40;
        public static final double SMART_CURRENT_LIMIT_DELAY = 0.2; // seconds
        public static final SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT_CONFIGURATION = new SupplyCurrentLimitConfiguration(
            true, 
            DrivetrainConstants.SMART_CURRENT_LIMIT, 
            DrivetrainConstants.HARD_CURRENT_LIMIT, 
            DrivetrainConstants.SMART_CURRENT_LIMIT_DELAY
        );
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
        public static final int HISTORY_LIMIT = 32;
        public static final double ENCODER_EPR = 2048;
        public static final double GEARING = (54./20.) * (50./12.);
        public static final boolean INVERT_GYRO = false;
        public static final double VELOCITY_DENOMINATOR = 0.1;
    }
    public static final class AutoConstants {
        public static final double KS = 0.68416;
        public static final double KV = 2.4928;
        public static final double KA = 0.3566;
        public static final double KP = 3.2643;
        public static final double TRACK_WIDTH = 0.419;

        public static final double MAXIMUM_VELOCITY = 1; // m/s
        public static final double MAXIMUM_ACCELERATION = 1; // m/s^2

        public static final DifferentialDriveKinematics DRIVETRAIN_KINEMATICS = new DifferentialDriveKinematics(
            TRACK_WIDTH);

        public static final RamseteConstants RAMSETE_CONSTANTS = new RamseteConstants(
            KS, KV, KA, KP, MAXIMUM_VELOCITY, MAXIMUM_ACCELERATION, DRIVETRAIN_KINEMATICS);
    }
    /*
    INTAKE PISTONS
    */
    public static final class IntakePistonsConstants {
        // solenoid channels
        public static final int FORWARD_CHANNEL = 0;
        public static final int REVERSE_CHANNEL = 3;
    }
    /*
    INTAKE
    */
    public static class IntakeConstants {
        // motor ids
        public static final int INTAKE_MOTOR = 20;

        public static final int STALL_CURRENT_LIMIT = 35;
        public static final int FREE_CURRENT_LIMIT = 30;
        public static final double SMART_CURRENT_LIMIT_DELAY = 0.2; // seconds
        public static final boolean MOTOR_INVERTED = false;

        public static final SimpleMotorFeedforward INTAKE_FF = new SimpleMotorFeedforward(0.10904, 3.9107, 0.081738);
        public static final PIDController PID = new PIDController(1.5841, 0, 0);
        public static final TrapezoidProfile.Constraints CONSTRAINTS = 
            new TrapezoidProfile.Constraints(5, 1);
        
        public static final double GEARING = 10;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(1);

        public static final double MIN_VELOCITY = 2;
    }
    public static final class ClimbConstants {

        public static final int OUTER_CLIMB_RIGHT_MOTOR = 50;
        public static final int OUTER_CLIMB_LEFT_MOTOR = 51;
        public static final int INNER_CLIMB_RIGHT_MOTOR = 55;
        public static final int INNER_CLIMB_LEFT_MOTOR = 56;

        public static final boolean OUTER_CLIMB_RIGHT_MOTOR_INVERTED = true;
        public static final boolean OUTER_CLIMB_LEFT_MOTOR_INVERTED = false;
        public static final boolean INNER_CLIMB_RIGHT_MOTOR_INVERTED = true; // false
        public static final boolean INNER_CLIMB_LEFT_MOTOR_INVERTED = true; // true
        public static final int STALL_CURRENT_LIMIT = 40;
        public static final int FREE_CURRENT_LIMIT = 35;

        // public static final double MIN_VELOCITY_FORWARD = 3;
        // public static final double MIN_VELOCITY_REVERSE = -5;

        // PID Constants
        // public static final PIDController OUTER_LEFT_PID = new PIDController(0, 0, 0);
        // public static final ElevatorFeedforward OUTER_LEFT_FF = new ElevatorFeedforward(0, 0, 0, 0);
        // public static final PIDController OUTER_RIGHT_PID = new PIDController(0, 0, 0);
        // public static final ElevatorFeedforward OUTER_RIGHT_FF = new ElevatorFeedforward(0, 0, 0, 0);
        // public static final PIDController INNER_LEFT_PID = new PIDController(0, 0, 0);
        // public static final ElevatorFeedforward INNER_LEFT_FF = new ElevatorFeedforward(0, 0, 0, 0);
        // public static final PIDController INNER_RIGHT_PID = new PIDController(0, 0, 0);
        // public static final ElevatorFeedforward INNER_RIGHT_FF = new ElevatorFeedforward(0, 0, 0, 0);

        public static final PIDController UNLOADED_ELEVATOR_PID = new PIDController(0.73455, 0, 0);
        public static final ElevatorFeedforward UNLOADED_ELEVATOR_FF = new ElevatorFeedforward(0.1956, -0.1091, 11.122, 0.53986);

        public static final PIDController LOADED_ELEVATOR_PID = new PIDController(79.132, 0, 0);
        public static final ElevatorFeedforward LOADED_ELEVATOR_FF = new ElevatorFeedforward(0.47132*2, 12.091*2, 0.95364*2, 0.821*2);

        // velocity and acceleration constraints
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(0.3, 1);
        // public static final double[] SOFT_LIMITS = {0.09, 0.6};
        public static final double GEARING = 12.0;
        public static final double SPOOL_RADIUS = 0.02;
        public static final double LOWER_LIMIT = 0;
        public static final double UPPER_lIMIT = Units.inchesToMeters(24.25);

        /*
        * Constants objects for climber arms. These must be declared here as they need to be
        * statically referenced and UNIQUE to each instance of the ClimberArm subsystem
        */
        // public static final FPIDConstants OUTER_LEFT_CONSTANTS = new FPIDConstants(GEARING, SPOOL_CIRCUMFERENCE,
        //     SOFT_LIMITS, ZERO_OFFSET_TICKS, ENCODER_CPR, OUTER_LEFT_PID, OUTER_LEFT_FF, CONSTRAINTS);
        // public static final FPIDConstants OUTER_RIGHT_CONSTANTS = new FPIDConstants(GEARING, SPOOL_CIRCUMFERENCE,
        //     SOFT_LIMITS, ZERO_OFFSET_TICKS, ENCODER_CPR, OUTER_RIGHT_PID, OUTER_RIGHT_FF, CONSTRAINTS);
        // public static final FPIDConstants INNER_LEFT_CONSTANTS = new FPIDConstants(GEARING, SPOOL_CIRCUMFERENCE,
        //     SOFT_LIMITS, ZERO_OFFSET_TICKS, ENCODER_CPR, INNER_LEFT_PID, INNER_LEFT_FF, CONSTRAINTS);
        // public static final FPIDConstants INNER_RIGHT_CONSTANTS = new FPIDConstants(GEARING, SPOOL_CIRCUMFERENCE,
        //     SOFT_LIMITS, ZERO_OFFSET_TICKS, ENCODER_CPR, INNER_RIGHT_PID, INNER_RIGHT_FF, CONSTRAINTS);
    }
    /*
    CONVEYOR
    */
    public static class ConveyorConstants {
        // motor ids
        public static final int CONVEYOR_MOTOR = 30;

        public static final boolean MOTOR_INVERTED = false;
        public static final int STALL_CURRENT_LIMIT = 35;
        public static final int FREE_CURRENT_LIMIT = 30;

        // PID Constants
        public static final SimpleMotorFeedforward CONVEYOR_FF = new SimpleMotorFeedforward(0.078977, 1.9659, 0.057779);
        public static final PIDController POSITION_PID = new PIDController(4.3574, 0, 0.12733);
        public static final PIDController VELOCITY_PID = new PIDController(0.17574, 0, 0);
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(2, 20); // TODO Check out high

        public static final int GEARING = 10;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
    }
    /*
    KICKER
    */
    public static class KickerConstants {
        // motor ids
        public static final int RIGHT_KICKER_MOTOR = 35;
        public static final int LEFT_KICKER_MOTOR = 36;

        public static final int STALL_CURRENT_LIMIT = 35;
        public static final int FREE_CURRENT_LIMIT = 30;

        public static final boolean RIGHT_KICKER_INVERTED = true;
        public static final boolean LEFT_KICKER_INVERTED = false;

        // PID Constants
        public static final SimpleMotorFeedforward RIGHT_KICKER_FF = new SimpleMotorFeedforward(0.15312, 1.9677, 0.27331);
        public static final SimpleMotorFeedforward LEFT_KICKER_FF = new SimpleMotorFeedforward(0.067363, 1.9044, 0.14625);
        public static final PIDController RIGHT_KICKER_PID = new PIDController(4.4195, 0, 0);
        public static final PIDController LEFT_KICKER_PID = new PIDController(3.5583, 0, 0);
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(2, 10); // TODO Should bethe same as conveyor

        public static final int GEARING = 10;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
    }
    public static final class TurretConstants {
        public static final int TURRET_MOTOR = 45;
        public static final boolean TURRET_MOTOR_INVERTED = false;
        public static final int STALL_CURRENT_LIMIT = 35;
        public static final int FREE_CURRENT_LIMIT = 30;

        // PID Constants
        public static final SimpleMotorFeedforward TURRET_FF = new SimpleMotorFeedforward(0.27429, 0.86838, 0.049705);
        public static final PID PID = new PID(9.65, 0, 1.0649);
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(6, 6);
        public static final int GEARING = 90;

        public static final double LIMIT = Units.degreesToRadians(90);
    }

    public static final class ClimbPistonsConstants {
        public static final int SET_1_FORWARD_CHANNEL = 1;
        public static final int SET_1_REVERSE_CHANNEL = 5;

        public static final int SET_2_FORWARD_CHANNEL = 2;
        public static final int SET_2_REVERSE_CHANNEL = 4;
    }
    /*
    SHOOTER
    */
    public static final class ShooterConstants {
        // motor ids
        public static final int SHOOTER_LEADER_MOTOR = 40;
        public static final int SHOOTER_FOLLOWER_MOTOR = 41;

        public static final boolean SHOOTER_LEADER_INVERTED = false;
        public static final boolean SHOOTER_FOLLOWER_INVERTED = true;
        public static final int SMART_CURRENT_LIMIT = 35;
        public static final int HARD_CURRENT_LIMIT = 40;
        public static final double SMART_CURRENT_LIMIT_DELAY = 0.2; // seconds
        public static final double SHOOTER_ANGLE = 70;
        public static final SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT_CONFIGURATION = new SupplyCurrentLimitConfiguration(
            true, 
            DrivetrainConstants.SMART_CURRENT_LIMIT, 
            DrivetrainConstants.HARD_CURRENT_LIMIT, 
            DrivetrainConstants.SMART_CURRENT_LIMIT_DELAY
        );

        // PID Constants
        public static final SimpleMotorFeedforward SHOOTER_FF = new SimpleMotorFeedforward(0.64689, 0.33756, 0.029811);
        public static final PIDController PID = new PIDController(.8, 0, 0);

        public static final double GEARING = 1;
        public static final double ENCODER_EPR = 2048;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
        public static final double VELOCITY_DENOMINATOR = 0.1;

        // limelight constants
        public static final double GOAL_HEIGHT_METERS = Units.inchesToMeters(103);//2.578;
        public static final double LENS_HEIGHT = 0.813;
        public static final double MOUNT_ANGLE = 30;
        public static final double DISTANCE_OFFSET = 0.7;

        public static final double DEFAULT_SPIN_UP_VELOCITY = 15;
    } 
}
