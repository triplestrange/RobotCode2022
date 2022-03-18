// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hood extends SubsystemBase {
  private final DoubleSolenoid hoodPiston;
  private boolean isExtended;
  /** Creates a new Hood. */
  public Hood() {
    hoodPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 7, 8);
    hoodPiston.set(Value.kReverse);

    addChild("Hood", hoodPiston);
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

  public boolean autoHood() {
    if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false)
    && NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0) < -15.0) {
      setHood(1);
      return true;
    } else {
      setHood(0);
      return false;
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("HoodExtended", getExtended());
  }
}
