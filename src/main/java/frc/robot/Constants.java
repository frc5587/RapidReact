// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.pid.PID;

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
    public static final class ShooterConstants {
        // motor ids
        public static final int SHOOTER_LEADER_MOTOR = 40;
        public static final int SHOOTER_FOLLOWER_MOTOR = 41;

        public static final boolean SHOOTER_LEADER_INVERTED = true; // TODO check if these need to be inverted
        public static final boolean SHOOTER_FOLLOWER_INVERTED = false; // TODO same here
        
        // motor speeds
        public static final int FORWARDS_VELOCITY = 1;
        public static final int BACKWARDS_VELOCITY = -1; // may not be needed, but implemented just so forwards & backwards can be different speeds
        
        // PID Constants
        // TODO Actually check what the PID values should be
        public static final PID PID = new PID(0, 0, 0);
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(0, 0);
        public static final double GEARING = 10;
        public static final double ENCODER_CPR = 42;
    } 
}
