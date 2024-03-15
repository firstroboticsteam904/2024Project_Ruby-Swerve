package frc.robot.commands.Auto.Climber;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AutoLiftUp extends Command {
    
    public AutoLiftUp(){

    }

    @Override
    public void initialize() {
        Robot.melanieClimber.set(Value.kForward);
        System.out.println("Climber Up");
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }


}
