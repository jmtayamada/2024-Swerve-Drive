package frc.robot;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.sensors.CANCoder;
import frc.robot.Constants.ModuleConstants;


public class SwerveModule {
    private final CANSparkMax m_drivingSparkMax;
    private final CANSparkMax m_turningSparkMax;

    private final RelativeEncoder m_drivingEncoder;
    private final CANCoder m_turningEncoder;

    private final SparkPIDController m_drivingPIDController;
    private final PIDController m_turningPIDController;

    // private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int drivingCANId, int turningCANId, int encoderNum) {
        m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

        m_drivingSparkMax.restoreFactoryDefaults();
        m_turningSparkMax.restoreFactoryDefaults();

        m_drivingEncoder = m_drivingSparkMax.getEncoder();
        m_turningEncoder = new CANCoder(encoderNum);

        m_turningPIDController = new PIDController(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD);
        m_drivingPIDController = m_drivingSparkMax.getPIDController();
        m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);

        m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

        m_turningPIDController.enableContinuousInput(ModuleConstants.kTurningEncoderPositionPIDMinInput, ModuleConstants.kTurningEncoderPositionPIDMaxInput);

        m_drivingPIDController.setP(ModuleConstants.kDrivingP);
        m_drivingPIDController.setI(ModuleConstants.kDrivingI);
        m_drivingPIDController.setD(ModuleConstants.kDrivingD);
        m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
        m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

        m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
        m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
        m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

        m_drivingSparkMax.burnFlash();
        m_turningSparkMax.burnFlash();
       
        m_drivingEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_drivingEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getAbsolutePosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_drivingEncoder.getPosition(), new Rotation2d(m_turningEncoder.getAbsolutePosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getAbsolutePosition() * Math.PI / 180));

        m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        m_turningSparkMax.set(m_turningPIDController.calculate(m_turningEncoder.getAbsolutePosition(), optimizedDesiredState.angle.getDegrees()));
    }

    public void testMotors(double velocity, double angle) {
        m_drivingPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
        m_turningSparkMax.set(m_turningPIDController.calculate(m_turningEncoder.getAbsolutePosition(), angle));
    }

    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }
}
