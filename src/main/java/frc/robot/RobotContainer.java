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
  private final ClimbMotor outerRightClimb = new ClimbMotor(Constants.ClimbConstants.OUTER_RIGHT_CONSTANTS);
  private final ClimbMotor innerLeftClimb = new ClimbMotor(Constants.ClimbConstants.INNER_LEFT_CONSTANTS);
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
    Trigger rightTrigger = new Trigger(() -> xboxController.getRightTriggerAxis() > 0);

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

    // aButton.and(leftTrigger).whenActive(() -> outerRightClimb.set(0.3), outerRightClimb)
    //     .whenInactive(outerRightClimb::stop, outerRightClimb);

    // bButton.and(leftTrigger).whenActive(() -> innerLeftClimb.set(0.3), innerLeftClimb)
    //     .whenInactive(innerLeftClimb::stop, innerLeftClimb);
    
    // Joystick control
    aButton.and(leftTrigger).whenActive(() -> outerRightClimb.set(xboxController.getLeftY()), outerRightClimb)
        .whenInactive(outerRightClimb::stop, outerRightClimb);

    bButton.and(leftTrigger).whenActive(() -> innerLeftClimb.set(xboxController.getLeftY()), innerLeftClimb)
        .whenInactive(innerLeftClimb::stop, innerLeftClimb);

    // PID CONTROL!!!! DO NOT USE UNTIL CHARACTERIZED
      // aButton.and(leftTrigger).whenActive(() -> outerRightClimb.setGoal(0.25), outerRightClimb)
      //     .whenInactive(outerRightClimb::stop, outerRightClimb);
  
      // bButton.and(leftTrigger).whenActive(() -> innerLeftClimb.setGoal(0.25), innerLeftClimb)
      //     .whenInactive(innerLeftClimb::stop, innerLeftClimb);

      // aButton.and(rightTrigger).whenActive(() -> outerRightClimb.setGoal(0.01), outerRightClimb)
      //     .whenInactive(outerRightClimb::stop, outerRightClimb);
  
      // bButton.and(rightTrigger).whenActive(() -> innerLeftClimb.setGoal(0.01), innerLeftClimb)
      //     .whenInactive(innerLeftClimb::stop, innerLeftClimb);
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