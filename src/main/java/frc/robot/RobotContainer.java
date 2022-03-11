// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

import org.frc5587.lib.control.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Controllers
  private final DeadbandJoystick joystick = new DeadbandJoystick(0, 1.5);
  private final DeadbandXboxController xb = new DeadbandXboxController(1);

  // Subsystems
  private static final Climb outerLeftClimb = Climb.createInnerRightArm();
  private static final Climb outerRightClimb = Climb.createOuterRightArm();
  private static final Climb innerLeftClimb = Climb.createInnerLeftArm();
  private static final Climb innerRightClimb = Climb.createOuterLeftArm();
  private final ClimbPistons climbPistons = new ClimbPistons();

  // Commands


  // Others

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      // Instantiate button bindings

      JoystickButton aButton = new JoystickButton(xb, DeadbandXboxController.Button.kA.value);
      JoystickButton bButton = new JoystickButton(xb, DeadbandXboxController.Button.kB.value);
      JoystickButton xButton = new JoystickButton(xb, DeadbandXboxController.Button.kX.value);
      JoystickButton yButton = new JoystickButton(xb, DeadbandXboxController.Button.kY.value);

      // Xbox Controller POV buttons
      POVButton dpadUp = new POVButton(xb, 0);
      POVButton dpadDown = new POVButton(xb, 180);

      // Xbox Controller triggers
      Trigger leftTrigger = new Trigger(() -> xb.getLeftTrigger());
      Trigger rightTrigger = new Trigger(() -> xb.getLeftTrigger());

      // Xbox Controller sticks
      Trigger leftStickY = new Trigger(() -> { 
        return xb.getLeftY() != 0;
      });
      Trigger leftStickX = new Trigger(() -> { 
        return xb.getLeftX() != 0;
      });
      Trigger rightStickY = new Trigger(() -> {
        return xb.getLeftY() != 0;
      });
      Trigger rightStickX = new Trigger(() -> {
        return xb.getLeftX() != 0;
      });

      leftStickX.and(rightTrigger)
        .whileActiveOnce(new OuterClimbThrottle(outerLeftClimb, outerRightClimb, xb::getLeftX));
      
      rightStickX.and(rightTrigger)
        .whileActiveOnce(new InnerClimbThrottle(innerLeftClimb, innerRightClimb, xb::getRightX));

      dpadUp.and(rightTrigger)
        .whileActiveOnce(new ParallelCommandGroup(
          new OuterClimbPosition(outerLeftClimb, outerRightClimb, -0.254)
// TODO FINISH CONTROL LOGIC
        ));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}