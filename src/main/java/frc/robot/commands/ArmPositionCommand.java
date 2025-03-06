package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
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

        System.out.printf("arm targetPos: ",targetPosition);
        System.out.printf("arm currentPos: ",currentPosition);
        System.out.printf("arm difference: ",difference);
        System.out.printf("arm tolerance: ",ArmConstants.TOLERANCE);

        return ArmConstants.TOLERANCE >= difference;
    }
}