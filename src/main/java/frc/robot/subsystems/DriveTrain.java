// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveTrain extends SubsystemBase 
{
  /** Creates a new ExampleSubsystem. */

    //create motor controller objects
    private static CANSparkMax leftLead = new CANSparkMax(2, MotorType.kBrushless);
    private static CANSparkMax rightLead = new CANSparkMax(3, MotorType.kBrushless);
    private static CANSparkMax leftFollower = new CANSparkMax(4, MotorType.kBrushless);
    private static CANSparkMax rightFollower = new CANSparkMax(5, MotorType.kBrushless);
  
    //create encoder objects
    static RelativeEncoder leftEncoder = leftLead.getEncoder();
    static RelativeEncoder rightEncoder = rightLead.getEncoder();
  
    // Create the differential drive object
    public final DifferentialDrive HamsterDrive = new DifferentialDrive(leftLead, rightLead);

  public DriveTrain() {
    // Set up the motor controller followers
    leftFollower.follow(leftLead);
    rightFollower.follow(rightLead);

    // Invert the right side motor controller
    rightLead.setInverted(true);

    //Disable the safety feature of the drivetrain, which can be very difficult to work around
    HamsterDrive.setSafetyEnabled(false);

    // Set deadband for the differential drive
    HamsterDrive.setDeadband(0.1);

    //Set the encoder positions to zero, effectively resetting them
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    //Set the motors to accelerate and decelerate slower
    rightLead.setOpenLoopRampRate(0.25);
    leftLead.setOpenLoopRampRate(0.25);
  }

  //#STOPDRIVE
  //Method to stop the drive train
  public void stopDrive() 
  {
    HamsterDrive.stopMotor();
  }


  Timer turnTimer = new Timer();

  //#AUTODRIVE
  //This method drives the auto for _ amount of time in a + or - direction
  public void autoDrive(double speed, double distance, double turn, double turnTime) 
  {
    rightDistance = 0;
    turnTimer.reset();
    turnTimer.start();
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);

    //Drive with positive distance
    if (distance > 0 && rightDistance < distance) 
    {
      HamsterDrive.arcadeDrive(speed, 0, false);
    } 
    else if (rightDistance >= distance) 
    {
      HamsterDrive.arcadeDrive(0, 0, false);
    }

    //Drive with negative distance
    if (distance < 0 && rightDistance > distance) 
    {
      HamsterDrive.arcadeDrive(-speed, 0, false);
    } 
    else if (rightDistance <= distance) 
    {
      HamsterDrive.arcadeDrive(0, 0, false);
    }

    //Turn
    if (turnTimer.get() < turnTime && turn != 0) 
    {
      HamsterDrive.arcadeDrive(0, turn, false);
    } 
    else if (turnTimer.get() >= turnTime && turn != 0) 
    {
      HamsterDrive.arcadeDrive(0, 0, false);
    }
  }

  public void reset(){
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);
  }


  static double rightWheelRotations = 0;
  static double leftWheelRotations = 0;

  static double rightDistance = 0;
  static double leftDistance = 0;

  //#ENCODERMATH
  //This function handles all of the math and data necessary to use the encoders
  public static void encoderMath() 
  {
    //All the math to convert encoder rotations to horizontal distance in inches
    rightWheelRotations = rightEncoder.getPosition() / 8.45;
    leftWheelRotations = leftEncoder.getPosition() / 8.45;

    rightDistance = rightWheelRotations * 18;
    leftDistance = leftWheelRotations * 18;

  }



  //#DRIVE
  //This method determines what to do with the motors based on the controller input
  public void drive(double forwardPow, double turnPow) 
  {
    //If the values are less than a certain point 
    // if (Math.abs(forwardPow) < .1) forwardPow = 0.0;
    // if (Math.abs(turnPow) < .1) turnPow = 0.0;
    //Move the robot.
    HamsterDrive.arcadeDrive(forwardPow, turnPow, true);
  }

  public void log(){
    // Displays the Left and Right encoder rates on the dashboard with the specified names
    SmartDashboard.putNumber("Left Encoder Distance", leftDistance);
    SmartDashboard.putNumber("Right Encoder Distance", rightDistance);
  }

  @Override
  public void periodic(){
    encoderMath();
    log();
  }

}
