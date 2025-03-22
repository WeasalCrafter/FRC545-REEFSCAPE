package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class CoralIntakeCommand extends Command{
    CoralIntakeSubsystem coralIntake;
    RobotContainer container;
    Double targetPosition;

    public CoralIntakeCommand(CoralIntakeSubsystem coralIntake, RobotContainer container){
        this.coralIntake = coralIntake;
        this.container = container;
        addRequirements(coralIntake);
    }

    @Override
    public void initialize(){
        coralIntake.changeSpeed(CoralIntakeConstants.SLOW_SPEED);
    }

    @Override
    public boolean isFinished() {
        // System.out.println(coralIntake.laserCan.getMeasurement().distance_mm);
        return (!container.getLaserCan().hasCoral());
    }
}
