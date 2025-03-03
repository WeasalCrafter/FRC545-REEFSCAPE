package frc.robot.subsystems;

<<<<<<< HEAD
=======
import com.revrobotics.AbsoluteEncoder;
>>>>>>> 7708ac7 (sunday)
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.Command;
=======

>>>>>>> 7708ac7 (sunday)
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
<<<<<<< HEAD

  /*
    * Allows for control of the elevator with two motors, one is the leader(leader) and performs 
    * PID calculations while the other(follower) is inverted and follows the leader motor(leader)
    * https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master
  */
=======
  // https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master
>>>>>>> 7708ac7 (sunday)

  public SparkMax m_leader; 
  public SparkMax m_follower;

<<<<<<< HEAD
=======
  public AbsoluteEncoder m_encoder;

>>>>>>> 7708ac7 (sunday)
  public SparkMaxConfig leaderMotorConfig;
  public SparkMaxConfig followerMotorConfig;

  public SparkClosedLoopController closedLoopController;
  public RelativeEncoder leader_encoder;

  public ElevatorSubsystem() {
    m_leader = new SparkMax(ElevatorConstants.VERT_LEADER_ID, MotorType.kBrushless);
    m_follower = new SparkMax(ElevatorConstants.VERT_FOLLOWER_ID, MotorType.kBrushless);
<<<<<<< HEAD

    closedLoopController = m_leader.getClosedLoopController();
    leader_encoder = m_leader.getEncoder();
    leaderMotorConfig = new SparkMaxConfig();
    leaderMotorConfig
        .inverted(true);
    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    leaderMotorConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

    leaderMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    // Set PID values for position control. We don't need to pass a closed loop
    // slot, as it will default to slot 0.
    .p(ElevatorConstants.PID_CONSTANTS.kP)
    .i(ElevatorConstants.PID_CONSTANTS.kI)
    .d(ElevatorConstants.PID_CONSTANTS.kD)
    .outputRange(-1, 1)
    // Set PID values for velocity control in slot 1
    .p(0.0001, ClosedLoopSlot.kSlot1)
    .i(0, ClosedLoopSlot.kSlot1)
    .d(0, ClosedLoopSlot.kSlot1)
    .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
    .outputRange(ElevatorConstants.MIN_SPEED, ElevatorConstants.MAX_SPEED, ClosedLoopSlot.kSlot1);

        /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    m_leader.configure(leaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize and configure the follower/follower motor
    followerMotorConfig = new SparkMaxConfig();
=======
    m_encoder = m_leader.getAbsoluteEncoder();
    followerMotorConfig = new SparkMaxConfig();
    leaderMotorConfig = new SparkMaxConfig();

    closedLoopController = m_leader.getClosedLoopController();
    leader_encoder = m_leader.getEncoder();
    
    leaderMotorConfig
      .inverted(true);    
    leaderMotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);
    leaderMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(ElevatorConstants.PID_CONSTANTS.kP)
      .i(ElevatorConstants.PID_CONSTANTS.kI)
      .d(ElevatorConstants.PID_CONSTANTS.kD)
      .outputRange(-1, 1)
      .p(0.0001, ClosedLoopSlot.kSlot1)
      .i(0, ClosedLoopSlot.kSlot1)
      .d(0, ClosedLoopSlot.kSlot1)
      .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
      .outputRange(ElevatorConstants.MIN_SPEED, ElevatorConstants.MAX_SPEED, ClosedLoopSlot.kSlot1);
    m_leader.configure(leaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

>>>>>>> 7708ac7 (sunday)
    followerMotorConfig
        .inverted(false)
        .follow(ElevatorConstants.VERT_LEADER_ID);
    m_follower.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
<<<<<<< HEAD
  }


  public void changePosition(Double targetPosition){
      closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public Command elevatorPos0(){
    return this.runOnce(() -> changePosition(ElevatorConstants.POS_ZERO));
  }
  public Command elevatorPos1(){
    return this.runOnce(() -> changePosition(ElevatorConstants.POS_ONE));
  }
  public Command elevatorPos2(){
    return this.runOnce(() -> changePosition(ElevatorConstants.POS_TWO));
  }
  public Command elevatorPos3(){
    return this.runOnce(() -> changePosition(ElevatorConstants.POS_THREE));
  }
  public Command elevatorPos4(){
    return this.runOnce(() -> changePosition(ElevatorConstants.POS_FOUR));
=======

    closedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }


  public void setPosition(Double targetPosition){
      closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public double getPosition(){
    return m_encoder.getPosition();
>>>>>>> 7708ac7 (sunday)
  }
}