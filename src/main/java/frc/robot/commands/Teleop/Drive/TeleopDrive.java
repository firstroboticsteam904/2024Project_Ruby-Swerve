// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop.Drive;

import java.lang.module.ModuleDescriptor.Requires;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve.DriveSubsystem;

public class TeleopDrive extends Command {

  private final DriveSubsystem swerveDrive;
  private final Supplier<Double> xSpeedFunc, ySpeedFunc, rotationFunc;
  private final Supplier<Boolean> fieldOrientedFunc;
  private final SlewRateLimiter xRateLimiter, yRateLimiter, rotationRateLimiter;

  /** Creates a new TeleopDrive. */
  public TeleopDrive(DriveSubsystem swerveDrivetrain, Supplier<Double> xSpeed,
  Supplier<Double> ySpeed, Supplier<Double> rotationSpeed, Supplier<Boolean> fieldOriented) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDrive = swerveDrivetrain;
    xSpeedFunc = xSpeed;
    ySpeedFunc = ySpeed;
    rotationFunc = rotationSpeed;
    fieldOrientedFunc = fieldOriented;
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
    
    double ForwardX = xSpeedFunc.get();
    ForwardX = Math.copySign(ForwardX, ForwardX);
    ForwardX = xRateLimiter.calculate(ForwardX) * DriveConstants.maxDriveSpeedMetersPerSec;

    double StrafeY = ySpeedFunc.get();
    StrafeY = Math.copySign(StrafeY, StrafeY);
    StrafeY = yRateLimiter.calculate(StrafeY) * DriveConstants.maxDriveSpeedMetersPerSec;

    double rotationZ = rotationFunc.get();
    rotationZ = Math.copySign(rotationZ * rotationZ, rotationZ);
    rotationZ = rotationRateLimiter.calculate(rotationZ) * DriveConstants.rotationMotorMaxSpeedRadPerSec;

    swerveDrive.drive(
      -ForwardX, 
      -StrafeY, 
      rotationZ, 
      fieldOrientedFunc.get());

    if(fieldOrientedFunc.get() == true){
      System.out.println("Field Oriented");
    } else {
      System.out.println("Robot Oriented");
    }


  }

  public double deadzoneInputs(double input) {
    if (Math.abs(input) < 0.095 ) {
      return 0.0;
    }
    return input;
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
