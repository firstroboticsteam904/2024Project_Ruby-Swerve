// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeFlipperCmd extends Command {

  /** Creates a new IntakeFlipperCmd. */
  public IntakeFlipperCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.intakeFlipperthingy.set(Value.kForward);
    System.out.println("Intake Flipper Extended");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
