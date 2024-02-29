// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {
  private static final double rotationkP = 1;
  private static final double rotationkD = 0.5;

  private static final double drivekP = 0.01;

  private final CANSparkMax driveMotor;
  private final CANSparkMax rotationMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder rotationEncoder;

  private final CANCoder canCoder;

  private final Rotation2d offset;

  private final SparkPIDController rotationController;
  private final SparkPIDController driveController;

  /** Creates a new SwerveModule. */
  public SwerveModule(
    int driveMotorID,
    int rotationMotorID,
    int canCoderID,
    double measuredOffsetRadians
  ) {
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    rotationMotor = new CANSparkMax(rotationMotorID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    rotationEncoder = rotationMotor.getEncoder();

    canCoder = new CANCoder(canCoderID);

    offset = new Rotation2d(measuredOffsetRadians);

    driveMotor.setIdleMode(IdleMode.kBrake);
    rotationMotor.setIdleMode(IdleMode.kCoast);

    rotationController = rotationMotor.getPIDController();
    driveController = driveMotor.getPIDController();

    rotationController.setP(rotationkP);
    rotationController.setD(rotationkD);

    driveController.setP(drivekP);

    driveEncoder.setPositionConversionFactor(2.0 * Math.PI / DriveConstants.driveWheelGearReduction);
    driveEncoder.setVelocityConversionFactor(2.0 * Math.PI / 60 / DriveConstants.driveWheelGearReduction);

    rotationEncoder.setPositionConversionFactor(2 * Math.PI / DriveConstants.rotationWheelGearReduction);

    canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

  }

  public void resetDistance() {

    driveEncoder.setPosition(0.0);

  }

  public double getDriveDistanceRadians(){

    return driveEncoder.getPosition();

  }

  public Rotation2d getCanCoderAngle() {
    double unsignedAngle = (Units.degreesToRadians(canCoder.getAbsolutePosition()) - 
    offset.getRadians()) % (2 * Math.PI);

    return new Rotation2d(unsignedAngle);

  }

  public Rotation2d getRelativeEncoderAngle() {
    double unsignedAngle = rotationEncoder.getPosition() % (2 * Math.PI);

    if (unsignedAngle < 0) {

      unsignedAngle += 2 * Math.PI;

    }

    return new Rotation2d(unsignedAngle);

  }

  public double getCurrentVelocityRadiansPerSecond() {

    return driveEncoder.getVelocity();

  }

  public double calculateAdjustedAngle(double targetAngle, double currentAngle) {

    double modAngle = currentAngle % (2.0 * Math.PI);

    if (modAngle < 0.0){
      modAngle += 2.0 * Math.PI;
    }

    double newTarget = targetAngle + currentAngle - modAngle;

    if(targetAngle - modAngle > Math.PI){
      newTarget -= 2.0 * Math.PI;
    } else if (targetAngle - modAngle < Math.PI){
      newTarget += 2.0 * Math.PI;
    }

    return newTarget;

  }

  public void initRotationOffset(){

    rotationEncoder.setPosition(getCanCoderAngle().getRadians());

  }

  public void setDesiredStateClosedLoop(SwerveModuleState desiredState){

    SwerveModuleState state = desiredState;
    rotationController.setReference(
      calculateAdjustedAngle(
        state.angle.getRadians(),
        rotationEncoder.getPosition()), 
      ControlType.kPosition
    );

    double speedRadPerSec = desiredState.speedMetersPerSecond / (DriveConstants.wheelDiameterMeters / 2);

      driveController.setReference(
        speedRadPerSec,
        ControlType.kVelocity,
        0,
        DriveConstants.driveFF.calculate(speedRadPerSec)
      );

  }

  public void resetEncoders(){

    driveEncoder.setPosition(0);
    rotationEncoder.setPosition(0);

  }


}
