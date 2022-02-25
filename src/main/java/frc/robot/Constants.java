// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import org.frc5587.lib.pid.PID;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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

        public static final boolean LEFT_SIDE_INVERTED = false;
        public static final boolean RIGHT_SIDE_INVERTED = true;
        public static final boolean LEFT_ENCODERS_INVERTED = false;
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

        // auto stuff
        public static final double WHEEL_DIAMETER_METERS = 0.1524;
        public static final int HISTORY_LIMIT = 32;
        public static final int ENCODER_EPR = 2048;
        public static final double GEARING = (54/20) * (50/12);
        public static final boolean INVERT_GYRO = false;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR = 20;
        public static final boolean INVERTED = false;

        public static final SimpleMotorFeedforward INTAKE_FF = new SimpleMotorFeedforward(0.13591, 3.9213, 0.091014);
        public static final PID PID = new PID(1.9231, 0, 0);
        public static final TrapezoidProfile.Constraints CONSTRAINTS = 
            new TrapezoidProfile.Constraints(20, 100);
        public static final double GEARING = 10;
        public static final int ENCODER_CPR = 42;
    }

    public static final class ClimberArmConstants {
        // motor ports
        public static final int MOTOR = 25;

        public static final boolean MOTOR_INVERTED = true;
        // motor limits
        // public static final int STALL_LIMIT = 40;
        // public static final int FREE_LIMIT = 35;

        public static final double FORWARD_THROTTLE = 1;
        public static final double BACKWARDS_THROTTLE = 1;
    }
}
