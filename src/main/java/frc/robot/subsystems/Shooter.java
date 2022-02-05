// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  private final CANSparkMax shooter1, shooter2;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private final DoubleSolenoid hoodPiston;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, setpoint, cur_speed;
  private boolean isExtended;
  private NetworkTable table;
  
  /** Creates a new Shooter. */
  public Shooter() {
    shooter1 = new CANSparkMax(0, MotorType.kBrushless);
    shooter2 = new CANSparkMax(0, MotorType.kBrushless);

    shooter1.restoreFactoryDefaults();
    shooter1.setIdleMode(IdleMode.kCoast);
    shooter1.setSmartCurrentLimit(60);
    shooter1.burnFlash();
    shooter2.restoreFactoryDefaults();
    shooter2.setIdleMode(IdleMode.kCoast);
    shooter2.setSmartCurrentLimit(60);
    shooter2.burnFlash();
    shooter2.follow(shooter1);

    hoodPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    m_encoder = shooter1.getEncoder();

    kP = 1;
    kI = 0;
    kD = 1;
    kIz = 0;
    kFF = 1.0/5676.0;
    kMaxOutput = 1;
    kMinOutput = 0;
    maxRPM = 5676.0;

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    
    setpoint = 5000.0;

    // NetworkTables
    table = NetworkTableInstance.getDefault().getTable("shooter");
    table.getEntry("ShooterSetpoint").setDouble(5000.0);
    table.getEntry("ShooterP").setDouble(kP);
    table.getEntry("ShooterI").setDouble(kI);
    table.getEntry("ShooterD").setDouble(kD);
    
  }

  /**
   * @param double setpoint
   *    assigned speed for shooter
   * @return boolean var
   *    whether shooter speed is at the setpoint
   */
  public boolean atSpeed() {
    return (Math.abs(setpoint - m_encoder.getVelocity())) / (setpoint) < 0.07; 
  }

  public void setShooter() {
    m_pidController.setReference(setpoint, ControlType.kVelocity);
  }

  /**
   * Sets shooter1 speed to 0
   */
  public void stopShooter() {
    shooter1.set(0);
  }


  public void setHood(int pos) {
    if (pos == 0) {
      hoodPiston.set(Value.kReverse);
      setExtended(false);
    } else {
      hoodPiston.set(Value.kForward);
      setExtended(true);
    }
  }

  public boolean getExtended() {
    return isExtended;
  }

  public void setExtended(boolean val) {
    isExtended = val;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setpoint = table.getEntry("ShooterSetpoint").getDouble(5000.0);
    table.getEntry("HoodExtended").setBoolean(getExtended());
    table.getEntry("ShooterSpeed").setDouble(m_encoder.getVelocity());
  }
}
