// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.commands.Auto.Commands.DoNothing;
import frc.robot.commands.Auto.Commands.SpeakerShot;
import frc.robot.commands.Auto.Drive.DriveAuto;
import frc.robot.commands.Teleop.Climb.climbDownCmb;
import frc.robot.commands.Teleop.Climb.climbcommand;
import frc.robot.commands.Teleop.Drive.LimelightTracking;
import frc.robot.commands.Teleop.Drive.TeleopDrive;
import frc.robot.commands.Teleop.Drive.resetPigeon;
import frc.robot.commands.Teleop.Intake.IntakeCmdGroup;
import frc.robot.commands.Teleop.Intake.IntakeRestCmdGroup;
import frc.robot.commands.Teleop.Shooter.AmpRestCmd;
import frc.robot.commands.Teleop.Shooter.AmpScoringCmd;
import frc.robot.commands.Teleop.Shooter.NoteRotatorCmd;
import frc.robot.commands.Teleop.Shooter.ShootingCmdGroup;
import frc.robot.commands.Teleop.Shooter.ShootingCmdRestGroup;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem swerve = new DriveSubsystem();
  private final Shooter shoot = new Shooter();

  public final CommandXboxController DriverController = new CommandXboxController(0);
  private final CommandGenericHID OperatorController = new CommandGenericHID(1);

  private final Command doNothing = new DoNothing();
  private final Command backUp = new DriveAuto(swerve, 1.0, 0, 0, false);

  private final PIDController Seeza_LimelightPID = new PIDController(0.5, 0, 0.01);
  
  // Routine for shooting at speaker during autonomous.
  private final SequentialCommandGroup scgSpeakerShot = new SpeakerShot(swerve, shoot);

  private final SendableChooser<Command> m_Chooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    swerve.setDefaultCommand(
        new TeleopDrive(swerve,
            () -> DriverController.getLeftY(),
            () -> DriverController.getLeftX(),
            () -> -DriverController.getRightX(),
            () -> !DriverController.leftTrigger().getAsBoolean()));

    // Configure the trigger bindings
    configureBindings();

    m_Chooser = AutoBuilder.buildAutoChooser();

    NamedCommands.registerCommand("Shooter Command Group", new ShootingCmdGroup(shoot));
    NamedCommands.registerCommand("Shooting Rest Command Group", new ShootingCmdRestGroup(shoot));

    m_Chooser.setDefaultOption("Do Nothing", doNothing);
    m_Chooser.addOption("backUp", backUp);
    m_Chooser.addOption("speakerShot", scgSpeakerShot);
    m_Chooser.addOption("blue Amp", new PathPlannerAuto("BlueAmpAuto"));
    m_Chooser.addOption("Back&Rotate", new PathPlannerAuto("StraightBack&Rotate"));

    SmartDashboard.putData("autoChooser", m_Chooser);


  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // OperatorController.button(1).whileTrue(new ShooterCmdGroupTest(shoot));
    OperatorController.button(6).whileTrue(new ShootingCmdGroup(shoot));
    OperatorController.axisGreaterThan(2, 0.45).whileTrue(new IntakeCmdGroup(shoot));
    OperatorController.axisGreaterThan(3, 0.45).whileTrue(new IntakeCmdGroup(shoot));
    OperatorController.button(5).onTrue(new NoteRotatorCmd(shoot, 99));
    OperatorController.button(1).whileTrue(new AmpScoringCmd(shoot));

    OperatorController.button(6).whileFalse(new ShootingCmdRestGroup(shoot));
    OperatorController.axisGreaterThan(2, 0.44).whileFalse(new IntakeRestCmdGroup(shoot));
    OperatorController.axisGreaterThan(3, 0.44).whileFalse(new IntakeRestCmdGroup(shoot));
    OperatorController.button(1).whileFalse(new AmpRestCmd(shoot));

    DriverController.a().onTrue(new climbcommand());
    DriverController.b().onTrue(new climbDownCmb());
    DriverController.x().onTrue(new resetPigeon(swerve));

    DriverController.leftBumper().whileTrue(
      new LimelightTracking(swerve, 
      () -> DriverController.getLeftY(), 
      () -> DriverController.getLeftX(), 
      swerve.limelightCenter(), 
      false)
    );

    //No longer needed, was added to test "backUp" command without
    //constantly having to reset robot for Autonomous.
    /*
    DriverController.y().onTrue(backUp);
    DriverController.y().onFalse(new TeleopDrive(swerve,
        () -> DriverController.getLeftY(),
        () -> DriverController.getLeftX(),
        () -> -DriverController.getRightX(),
        () -> !DriverController.leftTrigger().getAsBoolean())
        );
*/

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_Chooser.getSelected();
    // return scgSpeakerShot;
    // return backUp;
  }
}
