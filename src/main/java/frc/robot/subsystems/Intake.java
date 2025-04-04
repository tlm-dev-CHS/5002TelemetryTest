package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
@Logged
public class Intake extends SubsystemBase {

    final SparkMax m_shooter = new SparkMax(OperatorConstants.m_armShooter, MotorType.kBrushless);

    SparkMaxConfig shooterConfig = new SparkMaxConfig();

    public Intake(){
        shooterConfig
            .idleMode(IdleMode.kBrake);

        m_shooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runIntake(double speed){
        m_shooter.setVoltage(speed);
    }

    public void stopIntake(){
        m_shooter.stopMotor();
    }

    public double getAmps(){
        return m_shooter.getOutputCurrent();
    }

    public BooleanSupplier gotCoral(){
        return ()->getAmps() >= 80;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake Amps", getAmps());
    }
    
}
