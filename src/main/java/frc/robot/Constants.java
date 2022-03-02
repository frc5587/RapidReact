// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import org.frc5587.lib.controllers.FFController;
import org.frc5587.lib.pid.PID;

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

        public static final int OUTER_CLIMB_RIGHT_MOTOR = 60;
        public static final int OUTER_CLIMB_LEFT_MOTOR = 61;
        public static final int INNER_CLIMB_RIGHT_MOTOR = 65;
        public static final int INNER_CLIMB_LEFT_MOTOR = 66;

        public static final boolean MOTOR_INVERTED = false;

        public static final double MIN_VELOCITY_FORWARD = 3;
        public static final double MIN_VELOCITY_REVERSE = -5;

        // PID Constants
        // TODO - Fill these in later
        public static final PID PID = new PID(0, 0, 0);
        public static final FFController FF_CONTROLLER = new FFController(0, 0, 0, 0, 0);
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(0, 0);
        public static final double[] SOFT_LIMITS = {0.0, 0.0};
        public static final double GEARING = 10.0;
        public static final int ENCODER_CPR = 42;
    }

    public static final class ClimbPistonsConstants {
        // TODO - CHANGE VALUES
        public static final int SET_1_FORWARD_CHANNEL = 1;
        public static final int SET_1_REVERSE_CHANNEL = 4;

        public static final int SET_2_FORWARD_CHANNEL = 2;
        public static final int SET_2_REVERSE_CHANNEL = 5;
    }
}
