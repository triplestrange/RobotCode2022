// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Imports from we
import frc.robot.Constants; 

//Imports from Rev Robotics Library
import com.revrobotics.CANSparkMax; 
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 

//Imports from WPILib
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
  /*Constants class has not yet been filled inwith constant values
    so constant.intake.motor gets angry for now*/
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.motor,MotorType.kBrushless);
  private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(null, 0,1);
  private boolean extended = false;
  private double speedIn;
  private double speedOut;

  public Intake() {
    super();
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.enableVoltageCompensation(11);
    intakeMotor.setSmartCurrentLimit(20);
    //Add code to connect to Glass, our new dashboard
    //that is replacing smart dashboard
    intakeMotor.burnFlash();
  }

  /*method to extend the intake
    input: joystick is ...
     auto is...

  */
  public void extend (Joystick joystick, boolean auto) {
    if (auto) {
      intakeSolenoid.set(Value.kForward);
      setExtended(true);
      intakeMotor.set(.5);
    }


  }

  //method to retract the intake
  public void retract () {}


  //Getter for the boolean variable extended
  public boolean getExtended() {
    return extended;
  }

  //setter for the boolean variable extended
  public void setExtended(boolean newExtended) {
    this.extended = newExtended;
  }


  /*IDK the point of this but it was in the offseason
  so i am puting it here
  may have been a goal that was never finished
  @Override protected void initDefaultCommand() {
    //  TODO Auto-Generated method stub

  } */
}
