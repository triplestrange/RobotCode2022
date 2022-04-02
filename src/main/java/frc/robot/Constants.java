// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int LIMELIGHT = 18;

    public static final class SwerveConstants {
      // Port which the steering encoders are plugged into
      public static final int FL_ENCODER = 0;
      public static final int FR_ENCODER = 1;
      public static final int BL_ENCODER = 2;
      public static final int BR_ENCODER = 3;

      // add offsets for absolute encoders
      public static final int FL_OFFSET = 0;
      public static final int FR_OFFSET = 0;
      public static final int BL_OFFSET = 0;
      public static final int BR_OFFSET = 0;

      public static final double kMaxSpeedMetersPerSecond = 4.5693;

      public static final boolean kGyroReversed = true;
      
      // encoder's aren't reversed
      public static final boolean frontLeftSteerEncoderReversed = false;
      public static final boolean backLeftSteerEncoderReversed = false;
      public static final boolean frontRightSteerEncoderReversed = false;
      public static final boolean backRightSteerEncoderReversed = false;

      // Distance between centers of right and left wheels on robot in meters
    public static final double kTrackWidth = 0.59055;
    //Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.59055;
    //Distance between front and back wheels on robot

    // kinematics constructor with module positions as arguments
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    }

    public static final class ModuleConstants {
      public static final double kMaxModuleAngularSpeedRadiansPerSecond = 10 * Math.PI;
      public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 10 * Math.PI;
  
      public static final double kDriveEncoderCPR = (6);
      public static final double kSteerEncoderCPR = ((100d/30)*9);
  
      // adjust for calibration
      // 2/25/21 - 0.12584
      public static final double kWheelDiameterMeters = .1016;
      public static final double kDriveEncoderDistancePerPulse =
          // Assumes the encoders are directly mounted on the wheel shafts
          (kWheelDiameterMeters * Math.PI) / (double) kDriveEncoderCPR;
  
      public static final double kSteerEncoderDistancePerPulse =
          // Assumes the encoders are on a 1:1 reduction with the module shaft.
          (2 * Math.PI) / (double) kSteerEncoderCPR;
  
      public static final double kAbsoluteFL = (2*Math.PI)/3.332;
      public static final double kAbsoluteFR = (2*Math.PI)/3.236;
      public static final double kAbsoluteBL = (2*Math.PI)/3.30;
      public static final double kAbsoluteBR = (2*Math.PI)/3.312;
     
      public static final int FL_ENCODER = 0;
      public static final int FR_ENCODER = 2;
      public static final int BL_ENCODER = 1;
      public static final int BR_ENCODER = 3;
  
      public final static double FL_ENC_OFFSET = 0; // 183
      public final static double FR_ENC_OFFSET = 0; // 179
      public final static double BL_ENC_OFFSET = 0; // 221
      public final static double BR_ENC_OFFSET = 0; // 241
    }
    
    public static final class AutoConstants {
      public static final double kMaxSpeedMetersPerSecond = 1.5;
      public static final double kMaxAccelerationMetersPerSecondSquared = 2;
      public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  
      // changing here -- try raising gains further
      public static final double kPXController = 2.8;
      public static final double kPYController = 2.8;
      public static final double kDXController = 0;
      public static final double kDYController = 0;
      public static final double kPThetaController = 3;
  
      // Constraint for the motion profilied robot angle controller
      public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
          kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  
    }

    public static final class Shooting {
      public static final Double yValsShort[] = {-22.12, -17.12, -13.18,-11.840997, -6.470101, 0.8};
      public static final Double speedShort[] = {3800.0, 3550.0, 3150.0, 3000.0, 3000.0, 2750.0};

      public static final Double yValsLong[] = {-19.01, -16.19, -10.65, -3.80};
      public static final Double speedLong[] = {3950.0, 3600.0, 3300.0, 3000.0};

      public static final double height = .864; // of limelight
      public static final double hub = 2.64;
    }

    public static final class Electrical {
      // Swerve Motor Controller CAN ID's
      public static final int FL_DRIVE = 6;
      public static final int FR_DRIVE = 13;
      public static final int BL_DRIVE = 5;
      public static final int BR_DRIVE = 14;
      public static final int FL_STEER = 7;
      public static final int FR_STEER = 12;
      public static final int BL_STEER = 4;
      public static final int BR_STEER = 15;

      public static final int climbL = 2;
      public static final int climbR = 3;

      public static final int shooter1 = 17;
      public static final int shooter2 = 16;

      public static final int intake = 9;

      public static final int conveyor1 = 10;
      
      public static final int conveyor2 = 11;

      public static final int turret = 8;

      public static final int botSensor = 1;
      public static final int topSensor = 0;

    }

}
