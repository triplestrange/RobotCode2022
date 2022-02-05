// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
  2/5/22
  Austin Irwin
  Code "adapted" (copied with minor changes)
  from 2021OffseasonCode for intake

  This class controls the intake of our robot and should:
    extend intake arms using solenoids
    retract intake arms using solenoids
    run wheels on the intake bars to grab and pull in
    cargo.
 
  methods contain booleans  input to indicate wether the robot
  is running in autonomous or tele-op.

  What is done:
all necesary methods are created with good reference from 2021
libraries imported


  What still needs done:
initialization of constants.
verification that methods are compatible wtih new robot.
debugging.
possible more, i dont know what i dont know.
replace smartDashboard with glass


*/
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 
import com.revrobotics.CANSparkMax; 
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//we wont be using smart dashboard. need to import glass
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




public class Intake extends SubsystemBase {

  //intializes local variables

    /*Constants class has not yet been filled inwith constant values
    so constant.intake.motor gets angry for now*/
  private final CANSparkMax intakeMotor 
    = new CANSparkMax(Constants.Intake.motor,MotorType.kBrushless);
  private final DoubleSolenoid intakeSolenoid 
    = new DoubleSolenoid(null, 0,1);
  private boolean extended = false;
  private double speedIn;
  private double speedOut;
  private double autoSpeed; //.5

  /*
   */
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
  input: joystick is assignming a joystick for intake input
    auto=true: robot is running in autonomouas
    auto=false: robot is taking controller input
  */
  public void extend (Joystick joystick, boolean auto) {
    if (auto) {
      intakeSolenoid.set(Value.kForward);
      setExtended(true);
      intakeMotor.set(autoSpeed); 
    }
    else {
      //we arent using smartdashboard, will have to change this
      //double modifier = SmartDasboard.getNumber("Intake Speed",0.45);
      intakeSolenoid.set(Value.kForward);
      setExtended(true);
      double speedIn = joystick.getRawAxis(3);
      double speedOut = joystick.getRawAxis(2);

      if (speedIn > 0.1)
          intakeMotor.set(speedIn*modifier);
      else if (speedOut > 0.1)
          intakeMotor.set(-speedOut);
      else
          intakeMotor.set(0);
      
    }


  }

  //method to retract the intake
  public void retract () {
    intakeSolenoid.set(Value.kReverse);
    setExtended(false);
  }

  public void runWheels(double speed, boolean auto) {
    if(auto){
      intakeMotor.set(autoSpeed);
    }
    else{
      intakeMotor.set(speed);}

  }


  /***********************getters and setters****************/
  
  //Getter for the boolean variable extended
  public boolean getExtended() {
    return extended;
  }

  //setter for the boolean variable extended
  public void setExtended(boolean Extended) {
    this.extended = Extended;
  }


  /*IDK the point of this but it was in the offseason
  so i am puting it here
  may have been a goal that was never finished
  @Override protected void initDefaultCommand() {
    //  TODO Auto-Generated method stub

  } */
}
