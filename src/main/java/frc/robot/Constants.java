// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import org.frc5587.lib.auto.RamseteCommandWrapper.RamseteConstants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
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
        public static final StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT_CONFIGURATION = new StatorCurrentLimitConfiguration(
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
        //TODO: CHARACTERIZE!!
        public static final double KS = 0.66175;
        public static final double KV = 2.4683;
        public static final double KA = 0.29586;
        public static final double KP = 3.2643;
        public static final double TRACK_WIDTH = 0.648; //note: this is the right number,,,, stop asking :zany_face:

        public static final double MAXIMUM_VELOCITY = 1; // m/s
        public static final double MAXIMUM_ACCELERATION = 1; // m/s^2

        public static final DifferentialDriveKinematics DRIVETRAIN_KINEMATICS = new DifferentialDriveKinematics(
            TRACK_WIDTH);

        public static final RamseteConstants RAMSETE_CONSTANTS = new RamseteConstants(
            KS, KV, KA, KP, MAXIMUM_VELOCITY, MAXIMUM_ACCELERATION, DRIVETRAIN_KINEMATICS);
    }
    public static final class ShooterConstants {
        // motor ids
        public static final int SHOOTER_LEADER_MOTOR = 40;
        public static final int SHOOTER_FOLLOWER_MOTOR = 41;

        public static final boolean SHOOTER_LEADER_INVERTED = true;
        public static final boolean SHOOTER_FOLLOWER_INVERTED = false;
        
        // PID Constants
        public static final SimpleMotorFeedforward SHOOTER_FF = new SimpleMotorFeedforward(0.69172, 0.33773, 0.02674);
        public static final PIDController PID = new PIDController(0.95298, 0, 0); // 6.4838
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
        public static final double GEARING = 1;
        public static final double ENCODER_EPR = 2048;
        public static final double VELOCITY_DENOMINATOR = 0.1;
    } 
    public static class ConveyorConstants {
        public static final int CONVEYOR_MOTOR = 30;
        public static final boolean MOTOR_INVERTED = true;

        // PID Constants
        public static final SimpleMotorFeedforward CONVEYOR_FF = new SimpleMotorFeedforward(0.077365, 1.9438, 0.081956);
        public static final PIDController PID = new PIDController(5.0211, 0, 0);
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(.3, .3);
        public static final int GEARING = 10;
        public static final int VELOCITY_FORWARD = 1;
        public static final int VELOCITY_REVERSE = -3;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
    }

    public static class KickerConstants {
        public static final int RIGHT_KICKER_MOTOR = 35;
        public static final int LEFT_KICKER_MOTOR = 36;

        public static final boolean RIGHT_KICKER_INVERTED = true;
        public static final boolean LEFT_KICKER_INVERTED = false;

        // PID Constants
        public static final SimpleMotorFeedforward RIGHT_KICKER_FF = new SimpleMotorFeedforward(0.15312, 1.9677, 0.27331);
        public static final SimpleMotorFeedforward LEFT_KICKER_FF = new SimpleMotorFeedforward(0.067363, 1.9044, 0.14625);
        public static final PIDController RIGHT_KICKER_PID = new PIDController(4.4195, 0, 0);
        public static final PIDController LEFT_KICKER_PID = new PIDController(3.5583, 0, 0);
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
        public static final int GEARING = 10;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
    }
        
    public static class IntakeConstants {
        public static final int INTAKE_MOTOR = 20;
        public static final boolean INVERTED = false;

        public static final double MIN_VELOCITY_FORWARD = 3;
        public static final double MIN_VELOCITY_REVERSE = -5;

        public static final SimpleMotorFeedforward INTAKE_FF = new SimpleMotorFeedforward(0.10904, 3.9107, 0.081738);
        public static final PIDController PID = new PIDController(1.5841, 0, 0);
        public static final TrapezoidProfile.Constraints CONSTRAINTS = 
            new TrapezoidProfile.Constraints(5, 1);
        
        public static final double GEARING = 10;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(1);
    }

    public static final class IntakePistonsConstants {
        public static final int FORWARD_CHANNEL = 0;
        public static final int REVERSE_CHANNEL = 3;
    }
}
