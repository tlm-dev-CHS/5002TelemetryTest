package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class Climber extends SubsystemBase {

    final SparkMax m_climber = new SparkMax(OperatorConstants.m_climber, MotorType.kBrushless);

    final RelativeEncoder encoder = m_climber.getEncoder();

    SparkMaxConfig climberConfig = new SparkMaxConfig();


    public Climber(){

        climberConfig
            .idleMode(IdleMode.kBrake);

        m_climber.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runClimber(double speed){
        m_climber.setVoltage(speed);
    }

    public void stop(){
        m_climber.stopMotor();
    }
    
}
