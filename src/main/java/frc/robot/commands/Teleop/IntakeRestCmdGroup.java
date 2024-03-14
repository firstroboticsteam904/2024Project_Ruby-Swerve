// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeRestCmdGroup extends SequentialCommandGroup {
private final Shooter shooter;
  /** Creates a new IntakeCmdGroup. */
  public IntakeRestCmdGroup(Shooter shooter) {
    this.shooter=shooter;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new intakeRestPosCmd(),
    new WaitCommand(0.4),
    new shootRestAngleCmd(),
    new WaitCommand(1),
    new TeleopStop(shooter)
    );
  }
}
