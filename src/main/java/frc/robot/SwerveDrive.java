package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants.DriveConstants;

public class SwerveDrive {
    private final SwerveModule m_FLSwerve;
    private final SwerveModule m_FRSwerve;
    private final SwerveModule m_RLSwerve;
    private final SwerveModule m_RRSwerve;

    private final Pigeon2 m_gyro;

    public SwerveDrive() {
        m_FLSwerve = new SwerveModule(DriveConstants.kFrontLeftDrivingCanId, DriveConstants.kFrontLeftTurningCanId, DriveConstants.kFrontLeftEncoderCanId);
        m_FRSwerve = new SwerveModule(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightEncoderCanId);
        m_RLSwerve = new SwerveModule(DriveConstants.kRearLeftDrivingCanId, DriveConstants.kRearLeftTurningCanId, DriveConstants.kRearLeftEncoderCanId);
        m_RRSwerve = new SwerveModule(DriveConstants.kRearRightDrivingCanId, DriveConstants.kRearRightTurningCanId, DriveConstants.kRearRightEncoderCanId);

        m_gyro = new Pigeon2(0);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(m_gyro.getAngle());
    }

    public double getTurnRate() {
        return (m_gyro.getRate()) * (((DriveConstants.kGyroReversed ? 1.0 : -1.0) - 0.5)/.5);
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
        m_FLSwerve.setDesiredState(desiredStates[0]);
        m_FRSwerve.setDesiredState(desiredStates[1]);
        m_RLSwerve.setDesiredState(desiredStates[2]);
        m_RRSwerve.setDesiredState(desiredStates[3]);
    }

    public void setX() {
        m_FLSwerve.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_FRSwerve.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_RLSwerve.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_RRSwerve.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void drive(double xSpeed, double ySpeed, double rot) {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_gyro.getAngle()));
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    public void PIDTuningHelper(double angle, double speed) {
        m_FLSwerve.testModule(angle, speed);
    }
}