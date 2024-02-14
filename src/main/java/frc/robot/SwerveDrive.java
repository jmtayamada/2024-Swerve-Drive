package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveDrive {
    private final SwerveModule m_FLSwerve;
    private final SwerveModule m_FRSwerve;
    private final SwerveModule m_RLSwerve;
    private final SwerveModule m_RRSwerve;

    private final Pigeon2 m_gyro;

    private StructArrayPublisher<SwerveModuleState> publisherRed;
    private StructArrayPublisher<SwerveModuleState> publisherBlue;

    public SwerveDrive() {
        m_FLSwerve = new SwerveModule(DriveConstants.kFrontLeftDrivingCanId, DriveConstants.kFrontLeftTurningCanId,
                DriveConstants.kFrontLeftEncoderCanId, ModuleConstants.FL_driveInverted,
                ModuleConstants.FL_steerInverted);
        m_FRSwerve = new SwerveModule(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId,
                DriveConstants.kFrontRightEncoderCanId, ModuleConstants.FR_driveInverted,
                ModuleConstants.FR_steerInverted);
        m_RLSwerve = new SwerveModule(DriveConstants.kRearLeftDrivingCanId, DriveConstants.kRearLeftTurningCanId,
                DriveConstants.kRearLeftEncoderCanId, ModuleConstants.RL_driveInverted,
                ModuleConstants.RL_steerInverted);
        m_RRSwerve = new SwerveModule(DriveConstants.kRearRightDrivingCanId, DriveConstants.kRearRightTurningCanId,
                DriveConstants.kRearRightEncoderCanId, ModuleConstants.RR_driveInverted,
                ModuleConstants.RR_steerInverted);

        m_gyro = new Pigeon2(DriveConstants.kPigeonGyro);
        publishStates();
    }

    public void publishStates() {
        publisherRed = NetworkTableInstance.getDefault()
                .getStructArrayTopic("/SwerveStates/Red", SwerveModuleState.struct)
                .publish();
        publisherBlue = NetworkTableInstance.getDefault()
                .getStructArrayTopic("/SwerveStates/Blue", SwerveModuleState.struct).publish();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(m_gyro.getAngle());
    }

    public double getTurnRate() {
        return (m_gyro.getRate()) * (((DriveConstants.kGyroReversed ? 1.0 : -1.0) - 0.5) / .5);
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public void resetEncoders() {
        m_FLSwerve.resetEncoders();
        m_FRSwerve.resetEncoders();
        m_RLSwerve.resetEncoders();
        m_RRSwerve.resetEncoders();
    }

    public void setModuleStates(edu.wpi.first.math.kinematics.SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        
        m_FLSwerve.runModuleOptimised(desiredStates[0]);
        m_FRSwerve.runModuleOptimised(desiredStates[1]);
        m_RLSwerve.runModuleOptimised(desiredStates[2]);
        m_RRSwerve.runModuleOptimised(desiredStates[3]);

        publisherRed.set(new SwerveModuleState[] {
                desiredStates[0], desiredStates[1], desiredStates[2], desiredStates[3]
        });
        publisherBlue.set(new SwerveModuleState[] { m_FLSwerve.getState(), m_FRSwerve.getState(), m_RLSwerve.getState(),
                m_RRSwerve.getState() });

    }

    public void setX() {
        m_FLSwerve.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(50)));
        m_FRSwerve.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_RLSwerve.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_RRSwerve.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void drive(double xSpeed, double ySpeed, double rot) {
        SmartDashboard.putNumber("Gyro Position", m_gyro.getAngle() / 180 * Math.PI);
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                Rotation2d.fromRadians((90-m_gyro.getAngle())*Math.PI/180));
        SmartDashboard.putNumber("gyro", m_gyro.getAngle());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    public void PIDTuningHelper(double angle, double speed) {
        m_FRSwerve.testModule(angle, speed);
        m_FLSwerve.testModule(angle, speed);
        m_RLSwerve.testModule(angle, speed);
        m_RRSwerve.testModule(angle, speed);
    }

    public void stopModules() {
        m_FRSwerve.stopModule();
        m_FLSwerve.stopModule();
        m_RLSwerve.stopModule();
        m_RRSwerve.stopModule();
    }
}