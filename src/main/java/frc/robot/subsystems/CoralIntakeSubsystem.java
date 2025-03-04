package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralIntakeConstants;

public class CoralIntakeSubsystem extends SubsystemBase{
    public SparkMax m_leader; 
    public SparkMax m_follower;

    public SparkMaxConfig leaderMotorConfig;
    public SparkMaxConfig followerMotorConfig;

    public DigitalInput hallEffect;

    public CoralIntakeSubsystem(){
        m_leader = new SparkMax(CoralIntakeConstants.LEADER, MotorType.kBrushless);
        m_follower = new SparkMax(CoralIntakeConstants.FOLLOWER, MotorType.kBrushless);
        hallEffect = new DigitalInput(CoralIntakeConstants.LIMIT);
        
        leaderMotorConfig = new SparkMaxConfig();
        leaderMotorConfig
        .inverted(false);
        m_leader.configure(leaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        followerMotorConfig = new SparkMaxConfig();
        followerMotorConfig
        .follow(CoralIntakeConstants.LEADER, true);

        m_follower.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void changeSpeed(Double targetSpeed){
        m_leader.set(targetSpeed);
    }

    public boolean getLimit(){
        return hallEffect.get();
    }

    public Command forward(){
        return this.runOnce(() -> changeSpeed(CoralIntakeConstants.SPEED));
    }
    
    public Command reverse(){
        return this.runOnce(() -> changeSpeed(-CoralIntakeConstants.SPEED));
    }
    
    public Command lock(){
        return this.runOnce(() -> changeSpeed(0.0));
    }
}
