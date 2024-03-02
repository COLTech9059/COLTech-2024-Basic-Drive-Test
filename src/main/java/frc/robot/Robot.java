// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.commands.DriveCommand;
// import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Manipulator;
// import java.util.function.DoubleSupplier;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //Autonomous command????
  private Command m_autonomousCommand;
  //Objects to be used in various robot functions.
  // private DriveTrain drivetrain = new DriveTrain();
  // private LimeLight limelight = new LimeLight();
  private Manipulator manipulator = new Manipulator();
  //Set up DriveCommand and its DoubleSuppliers.
  // private DoubleSupplier forward = () -> IO.dController.getLeftY();
  // private DoubleSupplier turn = () -> IO.dController.getRightX();
  // private DriveCommand dCommand = new DriveCommand(drivetrain, forward, turn);
  //Robot Container
  private RobotContainer m_RobotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // drivetrain.factoryReset();
    manipulator.initializeManipulator();

    m_RobotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    manipulator.manipulatorDashboard();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    manipulator.initializeManipulator();
    // dCommand.cancel();
    // drivetrain.reset();
    // limelight.stop();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // schedule the autonomous command (example)
    m_autonomousCommand = m_RobotContainer.getAutoCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    // limelight.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // limelight.runLimelight(drivetrain);
  }


  Timer driveTime = new Timer();
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // dCommand.schedule();

    // limelight.stop();
    driveTime.stop();
    driveTime.reset();
    driveTime.start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // drivetrain.drive();
    manipulator.controlManipulator();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
