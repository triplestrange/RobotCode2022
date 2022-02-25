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

      public static final boolean kGyroReversed = false;
      
      // encoder's aren't reversed
      public static final boolean frontLeftSteerEncoderReversed = false;
      public static final boolean backLeftSteerEncoderReversed = false;
      public static final boolean frontRightSteerEncoderReversed = false;
      public static final boolean backRightSteerEncoderReversed = false;

      // Distance between centers of right and left wheels on robot in meters
    public static final double kTrackWidth = 0.46355;
    //Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.71755;
    //Distance between front and back wheels on robot

    // kinematics constructor with module positions as arguments
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    }

    public static final class ModuleConstants {
      public static final double kMaxModuleAngularSpeedRadiansPerSecond = 10 * Math.PI;
      public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 10 * Math.PI;
  
      public static final double kDriveEncoderCPR = (8);
      public static final double kSteerEncoderCPR = ((100d/30)*10);
  
      // adjust for calibration
      // 2/25/21 - 0.12584
      public static final double kWheelDiameterMeters = .12935;
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
      public static final int FR_ENCODER = 1;
      public static final int BL_ENCODER = 2;
      public static final int BR_ENCODER = 3;
  
      public final static double FL_ENC_OFFSET = 0; // 183
      public final static double FR_ENC_OFFSET = 0; // 179
      public final static double BL_ENC_OFFSET = 0; // 221
      public final static double BR_ENC_OFFSET = 0; // 241
    }
    
    public static final class AutoConstants {
      public static final double kMaxSpeedMetersPerSecond = 1.5;
      public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
      public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  
      // changing here -- try raising gains further
      public static final double kPXController = 2.8;
      public static final double kPYController = 2.8;
      public static final double kDXController = 0;
      public static final double kDYController = 0;
      public static final double kPThetaController = 2;
  
      // Constraint for the motion profilied robot angle controller
      public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
          kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  
    }

    public static final class Electrical {
      // Swerve Motor Controller CAN ID's
      public static final int FL_DRIVE = 1;
      public static final int FR_DRIVE = 2;
      public static final int BL_DRIVE = 3;
      public static final int BR_DRIVE = 4;
      public static final int FL_STEER = 5;
      public static final int FR_STEER = 6;
      public static final int BL_STEER = 7;
      public static final int BR_STEER = 8;

      public static final int climbL = 9;
      public static final int climbR = 10;

      public static final int shooter1 = 11;
      public static final int shooter2 = 12;

      public static final int intake = 13;

      public static final int conveyor = 14;

      public static final int turret = 15;

      public static final int hopper = 16;
    }

}
