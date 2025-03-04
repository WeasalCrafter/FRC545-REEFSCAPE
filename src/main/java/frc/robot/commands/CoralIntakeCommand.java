package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class CoralIntakeCommand extends Command{
    CoralIntakeSubsystem coralIntake;
    Double targetPosition;

    public CoralIntakeCommand(CoralIntakeSubsystem coralIntake){
        this.coralIntake = coralIntake;
        addRequirements(coralIntake);
    }

    @Override
    public void initialize(){
        coralIntake.changeSpeed(CoralIntakeConstants.SPEED);
    }

    @Override
    public boolean isFinished() {
        return !coralIntake.getLimit();
    }
}
