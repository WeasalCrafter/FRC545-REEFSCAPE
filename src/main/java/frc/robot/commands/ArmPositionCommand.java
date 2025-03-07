package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPositionCommand extends Command{
    ArmSubsystem arm;
    Double targetPosition;

    public ArmPositionCommand(ArmSubsystem arm, double targetPosition){
        this.arm = arm;
        this.targetPosition = targetPosition;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
        arm.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        double currentPosition = arm.getPosition();
        double difference = Math.abs((targetPosition - currentPosition));

        // System.out.printf("elevator targetPos: ",targetPosition);
        // System.out.printf("elevator currentPos: ",currentPosition);
        
        // System.out.println(" ");
        // System.out.println("target: " + targetPosition);
        // System.out.println("current: " + currentPosition);
        // System.out.println("diff: " + difference);
        // System.err.println(" ");

        // System.out.printf("elevator tolerance: ",ElevatorConstants.TOLERANCE);

        return difference <= ArmConstants.TOLERANCE;
    }
}