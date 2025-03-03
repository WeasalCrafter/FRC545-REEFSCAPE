package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPositionCommand extends Command{
    ElevatorSubsystem elevator;
    Double targetPosition;

    public ElevatorPositionCommand(ElevatorSubsystem elevator, double targetPosition){
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        addRequirements(elevator);
    }

    @Override
    public void initialize(){
        elevator.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return ((elevator.getPosition() >= (targetPosition - ElevatorConstants.TOLERANCE)) && (elevator.getPosition() <= (targetPosition + ElevatorConstants.TOLERANCE)));
    }
}
