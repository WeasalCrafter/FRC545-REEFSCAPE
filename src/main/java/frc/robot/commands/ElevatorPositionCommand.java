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

    // @Override
    // public boolean isFinished() {
    //     System.out.println(elevator.getPosition());
    //     return ((elevator.getPosition() >= (targetPosition - ElevatorConstants.TOLERANCE)) && (elevator.getPosition() <= (targetPosition + ElevatorConstants.TOLERANCE)));
    // }

    @Override
    public boolean isFinished() {
        double currentPosition = elevator.getPosition();
        double difference = Math.abs((targetPosition - currentPosition));

        // System.out.printf("elevator targetPos: ",targetPosition);
        // System.out.printf("elevator currentPos: ",currentPosition);
        
        // System.out.println(" ");
        // System.out.println("target: " + targetPosition);
        // System.out.println("current: " + currentPosition);
        // System.out.println("diff: " + difference);
        // System.err.println(" ");

        // System.out.printf("elevator tolerance: ",ElevatorConstants.TOLERANCE);

        return difference <= ElevatorConstants.TOLERANCE;
    }
}
