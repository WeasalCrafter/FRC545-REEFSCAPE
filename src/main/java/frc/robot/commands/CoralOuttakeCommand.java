package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class CoralOuttakeCommand extends Command{
    CoralIntakeSubsystem coralIntake;
    RobotContainer container;
    Double targetPosition;

    public CoralOuttakeCommand(CoralIntakeSubsystem coralIntake, RobotContainer container){
        this.coralIntake = coralIntake;
        this.container = container;
        addRequirements(coralIntake);
    }

    @Override
    public void initialize(){
        coralIntake.changeSpeed(CoralIntakeConstants.HIGH_SPEED);
    }

    @Override
    public boolean isFinished() {
        // System.out.println(coralIntake.laserCan.getMeasurement().distance_mm);
        return (container.getLaserCan().hasCoral());
    }
}
