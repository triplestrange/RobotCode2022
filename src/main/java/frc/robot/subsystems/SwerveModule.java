// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  // motors
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_steerMotor;

  // controllers
  // private final SparkMaxPIDController pid;

  // encoders
  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_steerEncoder;
  private final AnalogInput m_absoluteEncoder;

  /**
   * Constructs a SwerveModule
   * 
   * @param driveChannel
   * @param steerChannel
   */
  public SwerveModule(int driveChannel, int steerChannel) {

    m_driveMotor = new CANSparkMax(driveChannel, MotorType.kBrushless);
    m_steerMotor = new CANSparkMax(steerChannel, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder(SparkMaxRelativeEncoder.kQuadrature);
    m_steerEncoder = m_steerMotor.getEncoder();
    m_absoluteEncoder = m_driveMotor.getEncoder(SparkMaxRelativeEncoder.kHallEffect);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
