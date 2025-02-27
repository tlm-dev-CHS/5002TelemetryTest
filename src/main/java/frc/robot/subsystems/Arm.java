package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.EncoderConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;

public class Arm extends SubsystemBase {
    

    final SparkMax m_armRotator = new SparkMax(OperatorConstants.m_armRotator, MotorType.kBrushless);
    

    SparkMaxConfig rotatorConfig = new SparkMaxConfig();
    

    final RelativeEncoder encoder = m_armRotator.getEncoder();
    final AbsoluteEncoder rotatorAbsoluteEncoder = m_armRotator.getAbsoluteEncoder();

    PIDController m_controller = new PIDController(0.5, 0, 0);

    double factor = 0.0;

    public Arm(){
        rotatorConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true)

        .encoder
            .positionConversionFactor(OperatorConstants.m_armConversionFactor)
            .velocityConversionFactor(OperatorConstants.m_armConversionFactor/60);
        
        m_armRotator.configure(rotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_controller.setTolerance(1);
    }

    public Command moveToPosition(Double position){
        return sequence(
            runOnce(()->{m_controller.setSetpoint(position);}),
            run(()->{runArm();}).until(atGoal())
        );
    }

    public double getMeasurement(){
        return encoder.getPosition();
    }

    public void runArm(){
        m_armRotator.set(m_controller.calculate(getMeasurement(), m_controller.getSetpoint()));
    }
    
    public void runMotor(double d){
        m_armRotator.set(d);
    }

    public BooleanSupplier atGoal(){
        return () -> (m_controller.atSetpoint());
    }

    public void stop(){
        m_armRotator.set(0.0);
    }

    public void calibrate(){
        factor = getMeasurement();
        factor = 90 / getMeasurement();

        rotatorConfig.alternateEncoder
                .positionConversionFactor(factor)
                .velocityConversionFactor(factor/60);

    }

    public void zero(){
        encoder.setPosition(0);
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Position", getMeasurement());
        SmartDashboard.putNumber("Arm Setpoint", m_controller.getSetpoint());
        SmartDashboard.putNumber("Arm Output", m_controller.calculate(getMeasurement()));
        SmartDashboard.putBoolean("Arm At Goal", m_controller.atSetpoint());
    }
}