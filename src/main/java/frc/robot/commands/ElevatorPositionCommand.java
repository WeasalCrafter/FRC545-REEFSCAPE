package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPositionCommand extends Command{
    ElevatorSubsystem elevator;
    RobotContainer container;
    CoralIntakeSubsystem coralIntake;
    Double targetPosition;

    public ElevatorPositionCommand(ElevatorSubsystem elevator, RobotContainer container, double targetPosition){
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        this.container = container;
        addRequirements(elevator);
    }

    @Override
    public void initialize(){
        if(container.getLaserCan().hasCoral()){
            // there is no coral
            elevator.setPosition(ElevatorConstants.POS_ONE);
        }else{
            // there is a coral
            elevator.setPosition(targetPosition);
        }
    }

    @Override
    public boolean isFinished() {
        double currentPosition = elevator.getPosition();
        double difference = Math.abs((targetPosition - currentPosition));

        return (difference <= ElevatorConstants.TOLERANCE);
    }
}
