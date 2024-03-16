// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auto.Shooter.AutoStop;
import frc.robot.commands.Auto.Shooter.ShootAutonomous;
import frc.robot.commands.Auto.Shooter.shootAngleAutoCmd;
import frc.robot.commands.Auto.Shooter.shootRestAngleAutoCmd;
import frc.robot.commands.Auto.Swerve.BlueAmpPath;
import frc.robot.commands.Auto.Swerve.BlueSourcePath;
import frc.robot.commands.Auto.Swerve.RedSourcePath;
import frc.robot.commands.Teleop.Shooter.NoteRotatorCmd;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedSourceSideAutoScore extends SequentialCommandGroup {
  /** Creates a new BlueAmpSideAutoScore. */
  private final Shooter shooter;
  private final DriveSubsystem swerve;
  public RedSourceSideAutoScore(Shooter shooter, DriveSubsystem swerve) {
    // Add your commands in the addCommands() call, e.g.
    this.shooter = shooter;
    this.swerve = swerve;
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new RedSourcePath(swerve),
      new WaitCommand(.6),
      new shootAngleAutoCmd(),
      new WaitCommand(.6),
      new ShootAutonomous(shooter),
      new ParallelCommandGroup(
        new WaitCommand(1.0),
        new NoteRotatorCmd(shooter, 99)
      ),
      new WaitCommand(.6),
      new ParallelCommandGroup(
        new AutoStop(shooter),
        new shootRestAngleAutoCmd()
      )


    );
  }
}
