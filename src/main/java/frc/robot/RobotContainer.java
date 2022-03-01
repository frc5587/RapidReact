// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.control.*;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final DeadbandXboxController xboxController = new DeadbandXboxController(1);

  // Subsystems
  private final OuterClimbMotors outerClimbMotors = new OuterClimbMotors();
  private final InnerClimbMotors innerClimbMotors = new InnerClimbMotors();
  private final ClimbPistons climbPistons = new ClimbPistons();

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
    // Instantiate controller bindings
    JoystickButton xButton = new JoystickButton(xboxController, XboxController.Button.kX.value);
    JoystickButton yButton = new JoystickButton(xboxController, XboxController.Button.kY.value);
    JoystickButton aButton = new JoystickButton(xboxController, XboxController.Button.kA.value);
    JoystickButton bButton = new JoystickButton(xboxController, XboxController.Button.kB.value);

    Trigger leftTrigger = new Trigger(() -> xboxController.getLeftTriggerAxis() > 0);

    // TODO - CHANGE BUTTON BINDINGS

    /**
     * CLIMB PISTONS
     */

    xButton.and(leftTrigger).whenActive(climbPistons::extendSet1, climbPistons).whenInactive(climbPistons::retractSet1,
        climbPistons);
        
    yButton.and(leftTrigger).whenActive(climbPistons::extendSet2, climbPistons).whenInactive(climbPistons::retractSet2,
        climbPistons);

    /**
     * CLIMB MOTORS
     */
    // TODO - Possibly add reverse???

    aButton.and(leftTrigger).whenActive(() -> outerClimbMotors.setSpeed(0.3), outerClimbMotors)
        .whenInactive(outerClimbMotors::stopClimb, outerClimbMotors);

    bButton.and(leftTrigger).whenActive(() -> innerClimbMotors.setSpeed(0.3), innerClimbMotors)
        .whenInactive(innerClimbMotors::stopClimb, innerClimbMotors);

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