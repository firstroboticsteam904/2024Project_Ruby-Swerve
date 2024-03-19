// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve.DriveSubsystem;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import frc.robot.commands.Auto.Commands.DoNothing;

import frc.robot.commands.Auto.Shooter.ShootAutonomous;
import frc.robot.commands.Auto.Shooter.shootRestAngleAutoCmd;
import frc.robot.commands.Teleop.Climb.climbDownCmb;
import frc.robot.commands.Teleop.Climb.climbcommand;
import frc.robot.commands.Teleop.Drive.TeleopDrive;
import frc.robot.commands.Teleop.Drive.resetPigeon;
import frc.robot.commands.Teleop.Intake.IntakeCmdGroup;
import frc.robot.commands.Teleop.Intake.IntakeRestCmdGroup;
import frc.robot.commands.Teleop.Intake.intakeRestPosCmd;
import frc.robot.commands.Teleop.Shooter.AmpRestCmd;
import frc.robot.commands.Teleop.Shooter.AmpScoringCmd;
import frc.robot.commands.Teleop.Shooter.NoteRotatorCmd;
import frc.robot.commands.Teleop.Shooter.RotatorStop;
import frc.robot.commands.Teleop.Shooter.ShootingCmdGroup;
import frc.robot.commands.Teleop.Shooter.ShootingCmdRestGroup;
import frc.robot.commands.Teleop.Shooter.shootRestAngleCmd;


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

  public final CommandXboxController DriverController = new CommandXboxController(0);
  private final CommandGenericHID OperatorController = new CommandGenericHID(1);

  private final Command doNothing = new DoNothing();


  private final SendableChooser<Command> m_Chooser;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
      
    swerve.setDefaultCommand(
      new TeleopDrive(swerve, 
      () -> DriverController.getLeftY(), 
      () -> DriverController.getLeftX(), 
      () -> -DriverController.getRightX(), 
      () -> !DriverController.leftTrigger().getAsBoolean())
    );




    // Configure the trigger bindings
    configureBindings();

    m_Chooser = AutoBuilder.buildAutoChooser();

    NamedCommands.registerCommand("Shooter Command Group", new ShootingCmdGroup(shoot));
    NamedCommands.registerCommand("Shooting Rest Command Group", new ShootingCmdRestGroup(shoot));

    SmartDashboard.putData("autoChooser", m_Chooser);

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

   //OperatorController.button(1).whileTrue(new ShooterCmdGroupTest(shoot));
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


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_Chooser.getSelected();
  }
}
