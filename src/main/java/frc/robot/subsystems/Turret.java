// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  private CANSparkMax motor;
  /** Creates a new Turret. */
  public Turret() {
    motor = new CANSparkMax(0, MotorType.kBrushless);

    motor.restoreFactoryDefaults();
    motor.setSmartCurrentLimit(30);
    motor.setIdleMode(IdleMode.kBrake);
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    motor.setSoftLimit(SoftLimitDirection.kForward, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
