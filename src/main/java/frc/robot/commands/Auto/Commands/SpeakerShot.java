// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or sharote it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auto.Drive.DriveAuto;
import frc.robot.commands.Auto.Shooter.ShootAutonomous;
import frc.robot.commands.Auto.Shooter.AutoStop;
import frc.robot.commands.Auto.Shooter.shootAngleAutoCmd;
import frc.robot.commands.Auto.Shooter.shootRestAngleAutoCmd;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpeakerShot extends SequentialCommandGroup {
  /** Creates a new SpeakerShot. */
  public SpeakerShot(DriveSubsystem swerveDrivetrain, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // TODO: May need to set isFieldOrented to true for this to function.
        new DriveAuto(swerveDrivetrain, 1.0, 0, 0, false),
        new shootAngleAutoCmd(),
        // TODO: Is this needed? Add in vision to line up the shot prior to shooting the Note.
        new WaitCommand(1),
        new ShootAutonomous(shooter),
        // TODO: May need to adjust this time based on how long it takes to shoot the Note.
        new WaitCommand(3),
        new AutoStop(shooter),
        new shootRestAngleAutoCmd()
    );
  }
}
