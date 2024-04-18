// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Drive;

import java.lang.module.ModuleDescriptor.Requires;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve.DriveSubsystem;

public class DriveAuto extends Command {

  private final DriveSubsystem swerveDrive;
  private final double xSpeedFunc;
  private final SlewRateLimiter xRateLimiter;

  /** Creates a new TeleopDrive. */
  public DriveAuto(DriveSubsystem swerveDrivetrain, double xSpeed,
  double ySpeed, double rotationSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDrive = swerveDrivetrain;
    xSpeedFunc = xSpeed;
    this.xRateLimiter = new SlewRateLimiter(3);
    addRequirements(swerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   //swerveDrive.resetDriveDistances();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

     SmartDashboard.putNumber("Distance Traveled", swerveDrive.getEncoderTicks());
    
    double ForwardX = xSpeedFunc;

    // TODO: Use isFieldOrented to reverse the X direction.
    swerveDrive.drive(
      -ForwardX,
      0.0,
      0.0,
false);

      swerveDrive.getEncoderTicks();


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
    if(swerveDrive.getEncoderTicks() > 47){
      // TODO: Use Math.abs() if X-axis can be inverted by the isFieldOrented.
      //if(Math.abs(swerveDrive.getEncoderTicks()) > 65){
      swerveDrive.resetDriveDistances();
      return true;
    }else{
    return false;
  }
}
}
