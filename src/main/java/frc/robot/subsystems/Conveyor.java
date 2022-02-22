// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Electrical;

public class Conveyor extends SubsystemBase {
  private CANSparkMax motor;
  private RelativeEncoder encoder;
  private double speed;
  private NetworkTable table;
  private DigitalInput sensor1 = new DigitalInput(9);
  private DigitalInput sensor2 = new DigitalInput(10);

  public Conveyor() {
    motor =  new CANSparkMax(Electrical.conveyor, MotorType.kBrushless);
    encoder = motor.getEncoder();

    motor.setIdleMode(IdleMode.kBrake);

    table = NetworkTableInstance.getDefault().getTable("conveyor");
  }

  public void runConveyor() {
    motor.set(speed);
  }

  public void stopConveyor() {
    motor.set(0);
  }

  public void autoIndexConveyor() {  
    if (sensor2.get()) {
      motor.set(0.5);
    } 
  }

  public void initDefaultCommand() {
    
  }

  @Override
  public void periodic() {
    speed = table.getEntry("ConveyorSetpoint").getDouble(0.7);
    table.getEntry("ConveyorSpeed").setDouble(encoder.getVelocity());
    table.getEntry("BallsDetectedTop").setBoolean(sensor1.get());
    table.getEntry("BallsDetectedBot").setBoolean(sensor2.get());
  }
}
