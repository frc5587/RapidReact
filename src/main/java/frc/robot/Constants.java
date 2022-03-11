// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import org.frc5587.lib.controllers.FFController;
import org.frc5587.lib.pid.PID;
import org.frc5587.lib.subsystems.FPIDSubsystem.FPIDConstants;

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
    public static final class ClimbConstants {

        public static final int OUTER_CLIMB_RIGHT_MOTOR = 50;
        public static final int OUTER_CLIMB_LEFT_MOTOR = 51;
        public static final int INNER_CLIMB_RIGHT_MOTOR = 55;
        public static final int INNER_CLIMB_LEFT_MOTOR = 56;

        public static final boolean OUTER_CLIMB_RIGHT_MOTOR_INVERTED = true;
        public static final boolean OUTER_CLIMB_LEFT_MOTOR_INVERTED = true;
        public static final boolean INNER_CLIMB_RIGHT_MOTOR_INVERTED = false;
        public static final boolean INNER_CLIMB_LEFT_MOTOR_INVERTED = false;

        public static final double MIN_VELOCITY_FORWARD = 3;
        public static final double MIN_VELOCITY_REVERSE = -5;

        // PID Constants
        public static final PIDController OUTER_LEFT_PID = new PIDController(0, 0, 0);
        public static final ElevatorFeedforward OUTER_LEFT_FF = new ElevatorFeedforward(0, 0, 0, 0);
        public static final PIDController OUTER_RIGHT_PID = new PIDController(0, 0, 0);
        public static final ElevatorFeedforward OUTER_RIGHT_FF = new ElevatorFeedforward(0, 0, 0, 0);
        public static final PIDController INNER_LEFT_PID = new PIDController(0, 0, 0);
        public static final ElevatorFeedforward INNER_LEFT_FF = new ElevatorFeedforward(0, 0, 0, 0);
        public static final PIDController INNER_RIGHT_PID = new PIDController(0, 0, 0);
        public static final ElevatorFeedforward INNER_RIGHT_FF = new ElevatorFeedforward(0, 0, 0, 0);

        // velocity and acceleration constraints
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(0.3, 0.2);
        public static final double[] SOFT_LIMITS = {0.09, 0.6};
        public static final double GEARING = 12.0;
        public static final int ENCODER_CPR = 42;
        public static final double SPOOL_CIRCUMFERENCE = 0.25; //meters
        public static final int ZERO_OFFSET_TICKS = 0;
        public static final double SPOOL_RADIUS = 2;

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

    public static final class ClimbPistonsConstants {
        // TODO - CHANGE VALUES
        public static final int SET_1_FORWARD_CHANNEL = 1;
        public static final int SET_1_REVERSE_CHANNEL = 5;

        public static final int SET_2_FORWARD_CHANNEL = 2;
        public static final int SET_2_REVERSE_CHANNEL = 4;
    }
}
