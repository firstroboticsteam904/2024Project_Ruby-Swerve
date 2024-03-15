// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class CANDevices {
    public static final int frontLeftDriveMotorID = 1;
    public static final int frontLeftRotationMotorID = 2;
    public static final int frontLeftAbsoluteEncoder = 3;

    public static final int frontRightDriveMotorID = 4;
    public static final int frontRightRotationMotorID = 5;
    public static final int frontRightAbsoluteEncoder = 6;

    public static final int backLeftDriveMotorID = 7;
    public static final int backLeftRotationMotorID = 8;
    public static final int backLeftAbsoluteEncoder = 9;

    public static final int backRightDriveMotorID = 10;
    public static final int backRightRotationMotorID = 11;
    public static final int backRightAbsoluteEncoder = 12;

    public static final int ShootMotor1 = 13;
    public static final int ShootMotor2 = 14;
    public static final int RotationMotor = 15;

    public static final int pigeonID = 26;
  }

  public static final class DriveConstants {
    //length from left to right
    public static final double trackWidth = Units.inchesToMeters(27.50);
    //length from front to back
    public static final double wheelBase = Units.inchesToMeters(28.25);

    public static final SwerveDriveKinematics kinematics = 
      new SwerveDriveKinematics(
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0), //front left
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), //front right
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), //back left
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) //back right
      );

    public static final double driveWheelGearReduction = 6.75;
    public static final double rotationWheelGearReduction = 21.43;

    public static final double wheelDiameterMeters = Units.inchesToMeters(4);

    public static final double rotationMotorMaxSpeedRadPerSec = 1.0;
    public static final double rotationMotorMaxAccelRadPerSec = 1.0;

    public static final SimpleMotorFeedforward driveFF = 
    new SimpleMotorFeedforward(0.254, 0.137);

    public static final double maxDriveSpeedMetersPerSec = Units.feetToMeters(15.1);
    public static final double teleopTurnRateDegPerSec = 360.0;


  }

  
    public static class DriverStationInfo{
        public static Optional<Alliance> allianceColor = DriverStation.getAlliance();
        public AllianceStationID allianceStationID = DriverStationJNI.getAllianceStation();
    }

  public static final class AutoConstants {

    public static final double maxVelMetersPerSec = 2;
    public static final double maxAccelMetersPerSec = 1;
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = //
    DriveConstants.rotationMotorMaxSpeedRadPerSec / 10;
public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
      new TrapezoidProfile.Constraints(
          kMaxAngularSpeedRadiansPerSecond,
          kMaxAccelerationMetersPerSecondSquared);

  }

  public static final class OperatorConstants{

    public static final double Deadzone = 0.095;
  }

}
