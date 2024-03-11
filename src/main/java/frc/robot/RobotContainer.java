// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Teleop.ShooterCmdGroupTest;
import frc.robot.commands.Teleop.TeleopDrive;
import frc.robot.commands.Teleop.TeleopIntake;
import frc.robot.commands.Teleop.TeleopShoot;
import frc.robot.commands.Teleop.TeleopStop;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem swerve = new DriveSubsystem();
  private final Shooter shoot = new Shooter();

  private final CommandXboxController DriverController = new CommandXboxController(0);
  private final CommandGenericHID OperatorController = new CommandGenericHID(1);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
      
    swerve.setDefaultCommand(
      new TeleopDrive(swerve, 
      () -> DriverController.getLeftY(), 
      () -> DriverController.getLeftX(), 
      () -> -DriverController.getRightX(), 
      () -> false)
    );


    // Configure the trigger bindings
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

   OperatorController.button(1).whileTrue(new ShooterCmdGroupTest(shoot));

    //OperatorController.button(2).whileTrue(new TeleopIntake(shoot));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An example command will be run in autonomous

  //}
}
