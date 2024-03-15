// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auto.Climber.AutoLiftDown;
import frc.robot.commands.Auto.Climber.AutoLiftUp;
import frc.robot.commands.Auto.Shooter.AutoStop;
import frc.robot.commands.Auto.Shooter.ShootAutonomous;
import frc.robot.commands.Auto.Shooter.shootAngleAutoCmd;
import frc.robot.commands.Auto.Shooter.shootRestAngleAutoCmd;
import frc.robot.commands.Auto.Swerve.BackUpAuto;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootCmdGroup extends SequentialCommandGroup {
  private final Shooter shooter;
  private final DriveSubsystem driveSubsystem;
  /** Creates a new ShootAndLift. */
  public ShootCmdGroup(Shooter shooter, DriveSubsystem driveSubsystem) {
    this.shooter = shooter;
    this.driveSubsystem = driveSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new shootAngleAutoCmd(),
      new WaitCommand(.6),
      new ShootAutonomous(shooter),
      new WaitCommand(.4),
      new AutoStop(shooter),
      new WaitCommand(.2),
      new shootRestAngleAutoCmd(),
      new BackUpAuto(driveSubsystem)
    );
  }
}
