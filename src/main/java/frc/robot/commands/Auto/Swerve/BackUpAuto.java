// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve.DriveSubsystem;

public class BackUpAuto extends Command {
  private final DriveSubsystem swerveDrive;
  /** Creates a new BackUpAuto. */
  public BackUpAuto(DriveSubsystem swerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrive = swerveDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConstants.maxVelMetersPerSec
    , AutoConstants.maxAccelMetersPerSec);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
      List.of(new Translation2d(1, 0),
        new Translation2d(2, 0)),
        new Pose2d(2, 1, Rotation2d.fromDegrees(180)),
        trajectoryConfig);

        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController RotatorController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, 
        AutoConstants.kThetaControllerConstraints);
        RotatorController.enableContinuousInput(-Math.PI, Math.PI);

            SwerveControllerCommand backUpTrajectory = 
            new SwerveControllerCommand(
      trajectory, 
      swerveDrive::getPose, 
      DriveConstants.kinematics, 
      xController,
      yController,
      RotatorController, 
      swerveDrive::setModuleStates, 
      swerveDrive);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
