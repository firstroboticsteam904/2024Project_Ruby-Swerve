// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private static final double frontLeftAngleOffset = Units.degreesToRadians(206.91);
  private static final double frontRightAngleOffset = Units.degreesToRadians(103.53);
  private static final double backLeftAngleOffset =  Units.degreesToRadians(4.48);
  private static final double backRightAngleOffset = Units.degreesToRadians(122.96);

  private SwerveDriveOdometry odometry;
  private Field2d field;


  private final SwerveModule frontLeft = 
    new SwerveModule(
      CANDevices.frontLeftDriveMotorID,
      CANDevices.frontLeftRotationMotorID,
      CANDevices.frontLeftAbsoluteEncoder,
      frontLeftAngleOffset);

         private final SwerveModule frontRight = 
    new SwerveModule(
      CANDevices.frontRightDriveMotorID,
      CANDevices.frontRightRotationMotorID,
      CANDevices.frontRightAbsoluteEncoder,
      frontRightAngleOffset);

         private final SwerveModule backLeft = 
    new SwerveModule(
      CANDevices.backLeftDriveMotorID,
      CANDevices.backLeftRotationMotorID,
      CANDevices.backLeftAbsoluteEncoder,
      backLeftAngleOffset);

         private final SwerveModule backRight = 
    new SwerveModule(
      CANDevices.backRightDriveMotorID,
      CANDevices.backRightRotationMotorID,
      CANDevices.backRightAbsoluteEncoder,
      backRightAngleOffset);


    private double commandedForward = 0;
    private double commandedStrafe = 0;
    private double commandedRotation = 0;

    private boolean isCommandedFieldRelative = false;

    private final PigeonIMU pigeon = new PigeonIMU(CANDevices.pigeonID);


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    pigeon.setYaw(0);

    frontLeft.initRotationOffset();
    frontRight.initRotationOffset();
    backLeft.initRotationOffset();
    backRight.initRotationOffset();

    frontLeft.resetDistance();
    frontRight.resetDistance();
    backLeft.resetDistance();
    backRight.resetDistance();

    
    odometry =
      new SwerveDriveOdometry(
      DriveConstants.kinematics, 
      getHeading(), 
      getModulePositions()
      );

    field = new Field2d();

    Shuffleboard.getTab("Odemetry").add(field);

    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetPose, 
      this::getChassisSpeeds, 
      this::RobotRelativeDrive,
      Constants.pathFollowerConfig,
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }, 
      this);

      PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

      SmartDashboard.putData("field", field);


  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getHeading(), getModulePositions());

    SmartDashboard.putNumber("heading", getHeading().getDegrees());
    SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getY());

    field.setRobotPose(getPose());

    double loggingState[] = {

    };

  }

  public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {
    commandedForward = forward;
    commandedStrafe = strafe;
    commandedRotation = rotation;

    isCommandedFieldRelative = isFieldRelative;

    ChassisSpeeds speeds =
    isFieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(
          forward, strafe, rotation, getHeading())
        : new ChassisSpeeds(forward, strafe, rotation);

    SwerveModuleState[] states = DriveConstants.kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxDriveSpeedMetersPerSec);

    setModuleStates(states);
  }

  public void RobotRelativeDrive(ChassisSpeeds robotRelativeSpeeds){
    ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = DriveConstants.kinematics.toSwerveModuleStates(targetChassisSpeeds);
    setModuleStates(targetStates);
  }

  public void setModuleStates(SwerveModuleState[] moduleStates) {

    frontLeft.setDesiredStateClosedLoop(moduleStates[0]);
    frontRight.setDesiredStateClosedLoop(moduleStates[1]);
    backLeft.setDesiredStateClosedLoop(moduleStates[2]);
    backRight.setDesiredStateClosedLoop(moduleStates[3]);

  }

  public SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states = {
    new SwerveModuleState(frontLeft.getCurrentVelocityRadiansPerSecond(), frontLeft.getRelativeEncoderAngle()),
    new SwerveModuleState(frontRight.getCurrentVelocityRadiansPerSecond(), frontRight.getRelativeEncoderAngle()),
    new SwerveModuleState(backLeft.getCurrentVelocityRadiansPerSecond(), backLeft.getRelativeEncoderAngle()),
    new SwerveModuleState(backRight.getCurrentVelocityRadiansPerSecond(), backRight.getRelativeEncoderAngle())
  };

  return states;

}

  public SwerveModulePosition[] getModulePositions() {

    SwerveModulePosition[] positions = {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
  };

  return positions;

}



  public Pose2d getPose() {

    return odometry.getPoseMeters();

  }

  public void resetPose(Pose2d InitialStartingPose) {

    pigeon.setYaw(0);
    odometry.resetPosition(getHeading(), getModulePositions(), InitialStartingPose);

  }

  public ChassisSpeeds getChassisSpeeds(){
    return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
  }


  public void resetDriveDistances() {

    frontLeft.resetDistance();
    frontRight.resetDistance();
    backLeft.resetDistance();
    backRight.resetDistance();

  }

  public double getAverageDriveDistanceRadians() {

    return ((
      Math.abs(frontLeft.getDriveDistanceRadians())
      + Math.abs(frontRight.getDriveDistanceRadians())
      + Math.abs(backLeft.getDriveDistanceRadians())
      + Math.abs(backRight.getDriveDistanceRadians())) / 4.0);

  }

  public double getAverageDriveVelocityRadiansPerSecond() {
    return ((
      Math.abs(frontLeft.getCurrentVelocityRadiansPerSecond())
      + Math.abs(frontRight.getCurrentVelocityRadiansPerSecond())
      + Math.abs(backLeft.getCurrentVelocityRadiansPerSecond())
      + Math.abs(backRight.getCurrentVelocityRadiansPerSecond())) / 4.0);

  }

  public Rotation2d getHeading(){
    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);

    return Rotation2d.fromDegrees(ypr[0]);
  }

  public double[] getCommandedDriveValues(){

    double[] values = {commandedForward, commandedStrafe, commandedRotation};

    return values;

  }

  public boolean getIsFieldRelative() {

    return isCommandedFieldRelative;

  }

  public void resetPigeon(){

    pigeon.setYaw(0);

  }
  
  public void stopModules(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public double getEncoderTicks(){
    double encoderticks = frontLeft.getEncoderTicks();
    return encoderticks;
  }


}
