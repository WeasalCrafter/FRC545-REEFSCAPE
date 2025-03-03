package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class CoralIntakeCommand extends Command{
    CoralIntakeSubsystem coralIntake;
    Double targetPosition;
    DigitalInput hallEffect;

    public CoralIntakeCommand(CoralIntakeSubsystem coralIntake){
        this.coralIntake = coralIntake;
        this.hallEffect = coralIntake.hallEffect;
        addRequirements(coralIntake);
    }

    @Override
    public void initialize(){
        coralIntake.changeSpeed(ElevatorConstants.INTAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
        return hallEffect.get();
    }
}
