// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class NoteRotatorCmd extends Command {
  private final Shooter shooter;
  
  double desiredrotatorticks;

  /** Creates a new NoteRotatorCmd. */
  public NoteRotatorCmd(Shooter shooter, double rotatorticks) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    desiredrotatorticks = rotatorticks;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //shooter.noteEncoder.setPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rotatorSpin = shooter.rotatorTravel();
    
    if(rotatorSpin <= desiredrotatorticks){
    shooter.rotatorSpeed(-.20);
  } else {
    shooter.rotatorSpeed(0);
  }
    //System.out.println("Rotator Spinning");
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(shooter.rotatorTravel() >=
    desiredrotatorticks
    ){
      shooter.noteEncoder.setPosition(0);
      shooter.rotatorSpeed(0);
      return true;
    } else {
      return false;
    }
  }
}
