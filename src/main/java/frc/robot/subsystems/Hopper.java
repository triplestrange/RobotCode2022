// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  private CANSparkMax motor;
  private RelativeEncoder encoder;
  private NetworkTable table;
  /** Creates a new Hopper. */
  public Hopper() {
    motor = new CANSparkMax(Constants.Electrical.hopper, MotorType.kBrushless);
    table = NetworkTableInstance.getDefault().getTable("hopper");
  }

  public void wheelsIn() {
    motor.set(0.5);
  }

  public void wheelsOut() {
    motor.set(-0.5);
  }

  public void stop() {
    motor.set(0);
  }

  @Override
  public void periodic() {
    table.getEntry("hopperSpeed").setDouble(encoder.getVelocity());
    // This method will be called once per scheduler run
  }
}
