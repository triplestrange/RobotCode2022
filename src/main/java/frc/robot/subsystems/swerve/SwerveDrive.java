/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Electrical;
import frc.robot.Constants.ModuleConstants;
// import frc.robot.Constants.ModuleConstants;
// import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveConstants;

@SuppressWarnings("PMD.ExcessiveImports")
public class SwerveDrive extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(Electrical.FL_DRIVE,
      Electrical.FL_STEER,
      ModuleConstants.FL_ENCODER,
      ModuleConstants.kAbsoluteFL,
      SwerveConstants.frontLeftSteerEncoderReversed,
      ModuleConstants.FL_ENC_OFFSET);

  private final SwerveModule m_rearLeft = new SwerveModule(Electrical.BL_DRIVE,
      Electrical.BL_STEER,
      ModuleConstants.BL_ENCODER,
      ModuleConstants.kAbsoluteBL,
      SwerveConstants.backLeftSteerEncoderReversed,
      ModuleConstants.BL_ENC_OFFSET);

  private final SwerveModule m_frontRight = new SwerveModule(Electrical.FR_DRIVE,
      Electrical.FR_STEER,
      ModuleConstants.FR_ENCODER,
      ModuleConstants.kAbsoluteFR,
      SwerveConstants.frontRightSteerEncoderReversed,
      ModuleConstants.FR_ENC_OFFSET);

  private final SwerveModule m_rearRight = new SwerveModule(Electrical.BR_DRIVE,
      Electrical.BR_STEER,
      ModuleConstants.BR_ENCODER,
      ModuleConstants.kAbsoluteBR,
      SwerveConstants.backRightSteerEncoderReversed,
      ModuleConstants.FR_ENC_OFFSET);

  private SwerveModuleState[] swerveModuleStates;
  private SwerveModuleState[] initStates;
  private double horPos;
  public ChassisSpeeds currentMovement;

  // The gyro sensor
  private static final Gyro navX = new AHRS(SPI.Port.kMXP);
  boolean gyroReset;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(SwerveConstants.kDriveKinematics, getAngle());

  SwerveDriveOdometry m_odometryTur = new SwerveDriveOdometry(SwerveConstants.kDriveKinematics, getAngle());

  /**
   * Creates a new DriveSubsystem.
   */
  public SwerveDrive() {
  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees((navX.getAngle() + 180) * (SwerveConstants.kGyroReversed ? 1.0 : -1.0));
  }

  public boolean getGyroReset() {
    return gyroReset;
  }

  public void setGyroReset(boolean gyroReset) {
    this.gyroReset = gyroReset;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        getAngle(),
        m_frontLeft.getState(),
        m_rearLeft.getState(),
        m_frontRight.getState(),
        m_rearRight.getState());
    SmartDashboard.putNumber("FLSteering", m_frontLeft.m_absoluteEncoder.getAngle());
    SmartDashboard.putNumber("FRSteering", m_frontRight.m_absoluteEncoder.getAngle());
    SmartDashboard.putNumber("BLSteering", m_rearLeft.m_absoluteEncoder.getAngle());
    SmartDashboard.putNumber("BRSteering", m_rearRight.m_absoluteEncoder.getAngle());
    SmartDashboard.putNumber("FLSteeringDeg", m_frontLeft.m_absoluteEncoder.getAngle() * 180.0 / Math.PI);
    SmartDashboard.putNumber("FRSteeringDeg", m_frontRight.m_absoluteEncoder.getAngle() * 180.0 / Math.PI);
    SmartDashboard.putNumber("BLSteeringDeg", m_rearLeft.m_absoluteEncoder.getAngle() * 180.0 / Math.PI);
    SmartDashboard.putNumber("BRSteeringDeg", m_rearRight.m_absoluteEncoder.getAngle() * 180.0 / Math.PI);
    SmartDashboard.putNumber("FLSteerNEO", m_frontLeft.m_turningEncoder.getPosition());
    SmartDashboard.putNumber("FRSteerNEO", m_frontRight.m_turningEncoder.getPosition());
    SmartDashboard.putNumber("BLSteerNEO", m_rearLeft.m_turningEncoder.getPosition());
    SmartDashboard.putNumber("BRSteerNEO", m_rearRight.m_turningEncoder.getPosition());
    SmartDashboard.putNumber("FLneo", m_frontLeft.getState().angle.getRadians());
    SmartDashboard.putNumber("FRneo", m_frontRight.getState().angle.getRadians());
    SmartDashboard.putNumber("BLneo", m_rearLeft.getState().angle.getRadians());
    SmartDashboard.putNumber("BRneo", m_rearRight.getState().angle.getRadians());
    SmartDashboard.putNumber("x", getPose().getTranslation().getX());
    SmartDashboard.putNumber("y", getPose().getTranslation().getY());
    SmartDashboard.putNumber("r", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("GYRO ANGLE", navX.getAngle());
    SmartDashboard.putNumber("TurnRate", getTurnRate());
    m_odometryTur.update(getAngle(),
        m_frontLeft.getState(),
        m_rearLeft.getState(),
        m_frontRight.getState(),
        m_rearRight.getState());

    SmartDashboard.putNumber("TurOdomX", m_odometryTur.getPoseMeters().getX());
    SmartDashboard.putNumber("TurOdomY", m_odometryTur.getPoseMeters().getY());
    SmartDashboard.putNumber("TurOdomR", m_odometryTur.getPoseMeters().getRotation().getRadians());

    double height = 0.864; // of limelight
    double hub = 2.64;
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
    // TODO: assign horizontal position
    if (SmartDashboard.getBoolean("Long Range", true)) {
      horPos = (hub - height) / Math.tan(Math.toRadians(ty + 35.5)) + 0.6;
    } else {
      horPos = (hub - height) / Math.tan(Math.toRadians(ty + 35.5 + 12)) + 0.6;
    }

    currentMovement = SwerveConstants.kDriveKinematics.toChassisSpeeds(
      m_frontLeft.getState(),
      m_rearLeft.getState(),
      m_frontRight.getState(),
      m_rearRight.getState());

    SmartDashboard.putNumber("goalDist", horPos);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public Pose2d getPoseTur() {
    return m_odometryTur.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, getAngle());
  }

  public void resetOdometryTur() {
    double turretPos = SmartDashboard.getNumber("TurretPos", 0.0);
    Pose2d pose = new Pose2d(horPos, 0, new Rotation2d(Math.toRadians(turretPos)));
    m_odometryTur.resetPosition(pose, getAngle());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    swerveModuleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
        SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
        SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    navX.reset();
    gyroReset = true;
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public static double getHeading() {
    double heading = Math.IEEEremainder(navX.getAngle(), 360) * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);
    SmartDashboard.putNumber("heading", heading);
    return heading;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navX.getRate() * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void zeroWheels() {
    m_frontLeft.resetWheel();
    m_rearLeft.resetWheel();
    m_frontRight.resetWheel();
    m_rearRight.resetWheel();
  }
}