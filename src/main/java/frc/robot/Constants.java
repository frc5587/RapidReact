// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import org.frc5587.lib.auto.RamseteCommandWrapper.RamseteConstants;
import org.frc5587.lib.pid.PID;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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
        public static final boolean RIGHT_ENCODERS_INVERTED = false;

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

        // auto
        public static final PID TURN_PID = new PID(0, 0, 0); // TODO: characterize this stuff lol
        public static final double TURN_PID_TOLERANCE_DEG = 0.5;
        public static final double TURN_PID_FORWARD_THROTTLE = 0;
        public static final double TURN_PID_UPDATE_PERIOD_SEC = 0;

        public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
        public static final int HISTORY_LIMIT = 32;
        public static final double ENCODER_EPR = 2048;
        public static final double GEARING = (54./20.) * (50./12.);
        public static final boolean INVERT_GYRO = false;
        public static final double VELOCITY_DENOMINATOR = 0.1;
    }
    public static final class AutoConstants {
        //TODO: CHARACTERIZE!!
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KP = 0;
        public static final double KD = 0;
        public static final double TRACK_WIDTH = 0.648; //note: this is the right number,,,, stop asking :zany_face:

        public static final double MAXIMUM_VELOCITY = 1; // m/s
        public static final double MAXIMUM_ACCELERATION = 1; // m/s^2

        public static final DifferentialDriveKinematics DRIVETRAIN_KINEMATICS = new DifferentialDriveKinematics(
            TRACK_WIDTH);

        public static final RamseteConstants RAMSETE_CONSTANTS = new RamseteConstants(
            KS, KV, KA, KP, MAXIMUM_VELOCITY, MAXIMUM_ACCELERATION, DRIVETRAIN_KINEMATICS);
    }
}
