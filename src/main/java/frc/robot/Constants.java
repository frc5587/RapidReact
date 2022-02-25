// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.pid.PID;

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
    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR = 20;
        public static final boolean INVERTED = false;

        public static final double MIN_VELOCITY_FORWARD = 1;
        public static final double MIN_VELOCITY_REVERSE = -5;

        public static final SimpleMotorFeedforward INTAKE_FF = new SimpleMotorFeedforward(0.13591, 3.9213, 0.091014);
        public static final PID PID = new PID(1.9231, 0, 0);
        public static final TrapezoidProfile.Constraints CONSTRAINTS = 
            new TrapezoidProfile.Constraints(20, 100);
        
        public static final double GEARING = 10;
        public static final double ENCODER_CPR = 42;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(1);
    }

    public static final class IntakePistonsConstants {
        public static final int FORWARD_CHANNEL = 0;
        public static final int REVERSE_CHANNEL = 3;
    }
}
