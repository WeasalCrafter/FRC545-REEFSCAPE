package frc.robot.subsystems;

import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    SparkMax m_motor;
    SparkMaxConfig config;
    SoftLimitConfig limit;

    public ClimberSubsystem(){
        m_motor = new SparkMax(ClimberConstants.CLIMBER, MotorType.kBrushless);
        config = new SparkMaxConfig();
        limit = new SoftLimitConfig();
        
        limit.forwardSoftLimit(ClimberConstants.FORWARD_LIMIT);
        limit.reverseSoftLimit(ClimberConstants.REVERSE_LIMIT);
        limit.forwardSoftLimitEnabled(true);
        limit.reverseSoftLimitEnabled(true);

        // config.apply(limit);
        m_motor.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setVelocity(double velocity) {
        m_motor.set(velocity);
    }

    public Command lock(){
        return this.runOnce(() -> setVelocity(0));
    }
    public Command ascend(){
        return this.runOnce(() -> setVelocity(ClimberConstants.SPEED));
    }
    public Command descend(){
        return this.runOnce(() -> setVelocity(-ClimberConstants.SPEED));
    }
}
