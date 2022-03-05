// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        public static final boolean MOTOR_INVERTED = false;

        // PID Constants
        public static final SimpleMotorFeedforward CONVEYOR_FF = new SimpleMotorFeedforward(0.078977, 1.9659, 0.057779);
        public static final PIDController POSITION_PID = new PIDController(4.3574, 0, 0.12733);
        public static final PIDController VELOCITY_PID = new PIDController(0.17574, 0, 0);
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
