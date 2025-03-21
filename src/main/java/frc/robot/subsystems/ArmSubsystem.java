package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase{

  /*
    * Allows for control of the elevator with two motors, one is the leader(left) and performs 
    * PID calculations while the other(right) is inverted and follows the leader motor(left)
    * https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master
  */

  public SparkMax m_pivot;
  public SparkMaxConfig pivotConfig;

  public SparkClosedLoopController pivotController;
  public RelativeEncoder m_encoder;

  public ArmSubsystem() {
    m_pivot = new SparkMax(ArmConstants.PIVOT, MotorType.kBrushless);
    m_encoder = m_pivot.getEncoder();

    pivotController = m_pivot.getClosedLoopController();
    pivotConfig = new SparkMaxConfig();
    pivotConfig
        .inverted(false);
    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    pivotConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

    pivotConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    // Set PID values for position control. We don't need to pass a closed loop
    // slot, as it will default to slot 0.
    .p(ArmConstants.PID_CONSTANTS.kP)
    .i(ArmConstants.PID_CONSTANTS.kI)
    .d(ArmConstants.PID_CONSTANTS.kD)
    .outputRange(-0.1, 0.1)
    // Set PID values for velocity control in slot 1
    .p(0.0001, ClosedLoopSlot.kSlot1)
    .i(0, ClosedLoopSlot.kSlot1)
    .d(0, ClosedLoopSlot.kSlot1)
    .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
    .outputRange(ArmConstants.MIN_SPEED, ArmConstants.MAX_SPEED, ClosedLoopSlot.kSlot1);

    m_pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pivotController.setReference(ArmConstants.POS_UP, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void setPosition(Double targetPosition){
    pivotController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public double getPosition(){
    return m_encoder.getPosition();
  }
}