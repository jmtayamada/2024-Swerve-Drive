package frc.robot;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Random;


public class SwerveModule {
    private final CANSparkMax m_drivingSparkMax;
    private final CANSparkMax m_turningSparkMax;

    private final RelativeEncoder m_drivingEncoder;
    private final CANcoder m_turningEncoder;

    private final SparkPIDController m_drivingPIDController;
    private final PIDController m_turningPIDController;

    private final Timer clock;
    private boolean turning = false;
    private double currentTime = 0;
    private double startingDirection = 0;
    private Random rand = new Random();

    // private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int drivingCANId, int turningCANId, int encoderNum) {
        m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

        m_drivingSparkMax.restoreFactoryDefaults();
        m_turningSparkMax.restoreFactoryDefaults();

        m_drivingEncoder = m_drivingSparkMax.getEncoder();
        m_turningEncoder = new CANcoder(encoderNum);

        CANcoderConfiguration EncoderConfig = new CANcoderConfiguration();
        EncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        m_turningEncoder.getConfigurator().apply(EncoderConfig);

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

        clock = new Timer();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_drivingEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_drivingEncoder.getPosition(), new Rotation2d(m_turningEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getAbsolutePosition().getValueAsDouble()* 2 * Math.PI));

        m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        m_turningSparkMax.set(m_turningPIDController.calculate(m_turningEncoder.getAbsolutePosition().getValueAsDouble()*360, optimizedDesiredState.angle.getDegrees()));
    }

    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }

    public void testModule(double angle, double speed) {
        m_drivingPIDController.setReference(speed, CANSparkMax.ControlType.kVelocity);
        m_turningSparkMax.set(m_turningPIDController.calculate(m_turningEncoder.getAbsolutePosition().getValueAsDouble()*360, angle));
    }

    public void tuneTurningP() {
        if(!turning) {
            currentTime = 0;
            clock.reset();
            clock.start();
            startingDirection = -1;
            turning = true;
        }
        if(turning) {
            resetEncoders();
            testModule(rand.nextDouble(0, 180), 0);
            // check if velocity changed direction
            if(m_turningEncoder.getVelocity().getValueAsDouble() > startingDirection) {
                m_drivingPIDController.setP(m_drivingPIDController.getP()/2);
                turning = false;
            }
            // check time it took to get to set point
            if(m_turningPIDController.atSetpoint()) {
                currentTime = clock.get();
                clock.stop();
                if(currentTime > .5) {
                    m_drivingPIDController.setP(m_drivingPIDController.getP()*2);
                    turning = false;
                }
            }
            SmartDashboard.putNumber("Current P value", m_drivingPIDController.getP());
        }
    }
}
