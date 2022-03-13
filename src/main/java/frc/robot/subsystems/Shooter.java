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
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Electrical;

public class Shooter extends SubsystemBase {
  private final CANSparkMax shooter1, shooter2;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private final DoubleSolenoid hoodPiston;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, setpoint;
  private boolean isExtended;
  private NetworkTable table;
  
  /** Creates a new Shooter. */
  public Shooter() {
    shooter1 = new CANSparkMax(Electrical.shooter1, MotorType.kBrushless);
    shooter2 = new CANSparkMax(Electrical.shooter2, MotorType.kBrushless);
    
    m_pidController = shooter1.getPIDController();

    shooter1.restoreFactoryDefaults();
    shooter1.setIdleMode(IdleMode.kCoast);
    shooter1.setSmartCurrentLimit(60);
    shooter1.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    shooter2.restoreFactoryDefaults();
    shooter2.setIdleMode(IdleMode.kCoast);
    shooter2.setSmartCurrentLimit(60);
    shooter1.setInverted(true);
    shooter2.follow(shooter1, true);



    hoodPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 7, 8);
    hoodPiston.set(Value.kReverse);

    m_encoder = shooter1.getEncoder();

    kP = 0.0001;
    kI = 0.0000000;
    kD = 0.0000006;
    kIz = 0;
    // kFF = 0;
    kFF = 1.0/5000.0;
    kMaxOutput = 1;
    kMinOutput = 0;
    maxRPM = 5676.0;

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    
    setpoint = 5000.0;

    // NetworkTables
    table = NetworkTableInstance.getDefault().getTable("shooter");
    SmartDashboard.putNumber("ShooterSetpoint", 3500.0);
    kP = SmartDashboard.getNumber("ShooterP", 0.0);
    kI = SmartDashboard.getNumber("ShooterI", 0.0);
    kD = SmartDashboard.getNumber("ShooterD", 0.0);
    
    shooter2.burnFlash();
    shooter1.burnFlash();

    periodic();
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
    // where to set the speed for shooter if change needed
    setpoint = SmartDashboard.getNumber("ShooterSetpoint", 1000.0);
    m_pidController.setReference(setpoint, ControlType.kVelocity);
    shooter1.getEncoder().getVelocity();
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

  public void visionShoot() {
    // b/t +/- 24.85, +/-20.5 deg
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    // double tyScaled = (ty / (20.5 * 2));
    // m_pidController.setReference(tyScaled * 500 + 3500, ControlType.kVelocity);

    Double yVals[] = {0.0, 0.0};
    Double speed[] = {0.0, 0.0};

    double val = 0;
    for (int x = 0; x < yVals.length - 1; x++) {
      if (ty > yVals[x] && ty < yVals[x + 1]) {
        val = (ty - yVals[x]) / (yVals[x + 1] - yVals[x]) * (speed[x + 1] - (speed[x]) + speed[x]);
      }
    }

    if (ty < yVals[0]) {
      val = (ty - yVals[0]) / (yVals[1] - yVals[0]) * (speed[1] - (speed[0]) + speed[1]);
    } else if (ty > yVals[yVals.length - 1]) {
      val = (ty - yVals[yVals.length - 2]) / (yVals[yVals.length - 1] - yVals[yVals.length - 2]) * (speed[yVals.length - 2] - (speed[yVals.length - 1]) + speed[1]);
    }

  }

  
  public void test() {
    hoodPiston.set(Value.kForward);
  }

  public void toggleHood() {
    if (getExtended()) {
      setHood(0);
    } else {
      setHood(1);
    }
  }

  public boolean getExtended() {
    return isExtended;
  }

  public void setExtended(boolean val) {
    isExtended = val;
  }

  public void initDefaultCommand() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("HoodExtended", getExtended());
    SmartDashboard.putNumber("Output", shooter1.getAppliedOutput());
    SmartDashboard.putNumber("ShooterSpeed", m_encoder.getVelocity());
    kP = SmartDashboard.getNumber("kP", 1.0);
  }
}
