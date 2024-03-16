// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;

public class Shooter extends SubsystemBase {

  private final CANSparkMax slayShooter1 = new CANSparkMax(CANDevices.ShootMotor1, MotorType.kBrushless);
  private final CANSparkMax slayShooter2 = new CANSparkMax(CANDevices.ShootMotor2, MotorType.kBrushless);


  private final CANSparkMax noteRotator = new CANSparkMax(CANDevices.RotationMotor, MotorType.kBrushless);
  public RelativeEncoder noteEncoder = noteRotator.getEncoder();



  /** Creates a new Shooter. */
  public Shooter() {
  }

   public void shooterSpeed(double shootSpeed){
    slayShooter1.set(-shootSpeed);
    slayShooter2.set(shootSpeed);
  }

  public void rotatorSpeed(double rotateSpeed){
    noteRotator.set(-rotateSpeed);

    noteEncoder = noteRotator.getEncoder();
    SmartDashboard.putNumber("CanPosition", noteEncoder.getPosition());
    
  }

  public double rotatorTravel(){

    double rotatorTicks = noteEncoder.getPosition();

    return rotatorTicks;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
