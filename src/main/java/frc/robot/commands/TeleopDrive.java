// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDrive extends Command {

  private final DriveSubsystem swerveDrivetrain;
  private final Supplier<Double> xSpeedFunc, ySpeedFunc, rotationFunc;
  private final Supplier<Boolean> fieldOrientedFunc;
  private final SlewRateLimiter xRateLimiter, yRateLimiter, rotationRateLimiter;

  /** Creates a new TeleopDrive. */
  public TeleopDrive(DriveSubsystem swerveDrivetrain, Supplier<Double> xSpeedFunc,
  Supplier<Double> ySpeedFunc, Supplier<Double> rotationFunc, Supplier<Boolean> fieldOrientedFun) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrivetrain = swerveDrivetrain;
    this.xSpeedFunc = xSpeedFunc;
    this.ySpeedFunc = ySpeedFunc;
    this.rotationFunc = rotationFunc;
    this.fieldOrientedFunc = fieldOrientedFun;
    this.xRateLimiter = new SlewRateLimiter(3);
    this.yRateLimiter = new SlewRateLimiter(3);
    this.rotationRateLimiter = new SlewRateLimiter(3);
    addRequirements(swerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xSpeedFunc.get();
    double ySpeed = ySpeedFunc.get();
    double rotationSpeed = rotationFunc.get();

    xSpeed = Math.abs(xSpeed) > OperatorConstants.Deadzone ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OperatorConstants.Deadzone ? ySpeed : 0.0;
    rotationSpeed = Math.abs(rotationSpeed) > OperatorConstants.Deadzone ? rotationSpeed : 0.0;

    xSpeed = xRateLimiter.calculate(xSpeed) * DriveConstants.maxDriveSpeedMetersPerSec;
    ySpeed = yRateLimiter.calculate(ySpeed) * DriveConstants.maxDriveSpeedMetersPerSec;
    rotationSpeed = rotationRateLimiter.calculate(rotationSpeed) * DriveConstants.teleopTurnRateDegPerSec;

    swerveDrivetrain.drive(
      xSpeed, 
      ySpeed, 
      rotationSpeed, 
      fieldOrientedFunc.get());

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
