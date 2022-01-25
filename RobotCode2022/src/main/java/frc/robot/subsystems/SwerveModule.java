// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANAnalog;
import com.revrobotics.CANEncoder;



public class SwerveModule extends SubsystemBase {
    // motors
    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;
  
    // encoders
    final CANEncoder m_driveEncoder;
    final CANEncoder m_turningEncoder;
    final CANAnalog m_absoluteEncoder;

    public SwerveModule() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
