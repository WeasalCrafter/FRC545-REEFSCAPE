package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
  // https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master

  public SparkMax m_leader; 
  public SparkMax m_follower;

  public AbsoluteEncoder m_encoder;

  public SparkMaxConfig leaderMotorConfig;
  public SparkMaxConfig followerMotorConfig;

  public SparkClosedLoopController closedLoopController;

  public ElevatorSubsystem() {
    m_leader = new SparkMax(ElevatorConstants.LEADER, MotorType.kBrushless);
    m_follower = new SparkMax(ElevatorConstants.FOLLOWER, MotorType.kBrushless);
    m_encoder = m_leader.getAbsoluteEncoder();
    followerMotorConfig = new SparkMaxConfig();
    leaderMotorConfig = new SparkMaxConfig();

    closedLoopController = m_leader.getClosedLoopController();
    
    leaderMotorConfig
      .inverted(true);    
    leaderMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(ElevatorConstants.PID_CONSTANTS.kP)
      .i(ElevatorConstants.PID_CONSTANTS.kI)
      .d(ElevatorConstants.PID_CONSTANTS.kD)
      .outputRange(ElevatorConstants.MIN_SPEED, ElevatorConstants.MAX_SPEED);
    m_leader.configure(leaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    followerMotorConfig
        .follow(ElevatorConstants.LEADER, false);
    m_follower.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    closedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void setPosition(Double targetPosition){
      closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public double getPosition(){
    return m_encoder.getPosition();
  }
}