// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Electrical;

public class Conveyor extends SubsystemBase {
  private CANSparkMax motor1, motor2;
  private RelativeEncoder encoder1, encoder2;
  private double speed;
  private NetworkTable table;
  private DigitalInput sensor1;
  private DigitalInput sensor2;
  private ColorSensorV3 m_colorSensor;
  private ColorMatch m_colorMatcher;
  private Color kBlueTarget, kRedTarget, detectedColor;
  private ColorMatchResult match;
  private String colorString;

  public Conveyor() {
    motor1 = new CANSparkMax(Electrical.conveyor1, MotorType.kBrushless);
    encoder1 = motor1.getEncoder();

    motor2 = new CANSparkMax(Electrical.conveyor2, MotorType.kBrushless);
    encoder2 = motor2.getEncoder();

    sensor1 = new DigitalInput(Electrical.botSensor);
    sensor2 = new DigitalInput(Electrical.topSensor);

    motor1.setIdleMode(IdleMode.kBrake);
    motor2.setIdleMode(IdleMode.kBrake);

    motor1.setSmartCurrentLimit(30);
    motor2.setSmartCurrentLimit(30);
    motor1.burnFlash();
    motor2.burnFlash();

    table = NetworkTableInstance.getDefault().getTable("conveyor");


    // LiveWindow
    addChild("Sensor1", sensor1);
    addChild("Sensor2", sensor2);

    I2C.Port i2cPort = I2C.Port.kOnboard;

    m_colorSensor = new ColorSensorV3(i2cPort);
    m_colorMatcher = new ColorMatch();

    kBlueTarget = new Color(0.143, 0.427, 0.429);
    kRedTarget = new Color(0.561, 0.232, 0.114);

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);

    colorString = SmartDashboard.getString("DriverStation", "Red");

    periodic();
  }

  public void autoConveyor() {
    // top is sensor 1
    // bottom is sensor 2
    // ! means it has a value
    if (sensor1.get() && !sensor2.get()) {
      runConveyor(-1);
    }
    if (!sensor1.get() && sensor2.get()) {
      motor1.set(-1);
      motor2.set(0);
    }
    if (sensor1.get() && sensor2.get()) {
      runConveyor(-1);
    }
    if (!sensor1.get() && !sensor2.get()) {
      stopConveyor();
    }
  }


  public void runConveyor() {
    motor1.set(-0.65);
    motor2.set(1);
  }

  public void runConveyor(double newSpeed) {
    motor1.set(newSpeed);
    motor2.set(-newSpeed);
  }

  public void stopConveyor() {
    motor1.set(0);
    motor2.set(0);
  }

  public boolean getTopSensor() {
    return !sensor1.get();
  }

  public boolean getBotSensor() {
    return !sensor2.get();
  }

  public boolean empty() {
    return !sensor1.get() == false && !sensor2.get() == false;
  }


  public void initDefaultCommand() {

  }

  @Override
  public void periodic() {   
    detectedColor = m_colorSensor.getColor();
    match = m_colorMatcher.matchClosestColor(detectedColor);

    if (detectedColor.blue > 0.3) {
      colorString = "Blue";
    } 
    if (detectedColor.red > 0.3) {
      colorString = "Red";
    } 
    SmartDashboard.getNumber("ConveyorSetpoint", 0.7);
    SmartDashboard.putBoolean("BotEmpty", empty());
    SmartDashboard.getNumber("ConveyorSpeed", encoder1.getVelocity());
    SmartDashboard.getNumber("HopperSpeed", encoder2.getVelocity());
    SmartDashboard.putBoolean("Top Sensor", !sensor1.get());
    SmartDashboard.putBoolean("Bottom Sensor", !sensor2.get());
    // SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    SmartDashboard.putString("DriverStation", DriverStation.getAlliance().toString());
  }
}
