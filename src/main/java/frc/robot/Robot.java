// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.CoralShootCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.PixelFormat;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private boolean autoDone = false; //simplistic code
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    try {
      UsbCamera camera = CameraServer.startAutomaticCapture(0);
      camera.setVideoMode(PixelFormat.kMJPEG,160,120,30);
    } catch (Exception e) {
      System.out.println("level 2 camera failed to initialize");
    }
    try {
      UsbCamera camera2 = CameraServer.startAutomaticCapture(1);
      camera2.setVideoMode(PixelFormat.kMJPEG,160,120,30);
    } catch (Exception e) {
      System.out.println("level 1 camera failed to initialize");
    }

    //m_chooser.setDefaultOption("Simple", "Simple");
    //m_chooser.setDefaultOption("Left", "Left");
    m_chooser.setDefaultOption("Simple", "Simple");
    m_chooser.addOption("Simple Long", "Simple Long");
    //m_chooser.addOption("Left", "Left");
    //m_chooser.addOption("Center", "Center");
    //m_chooser.addOption("Right", "Right");
    SmartDashboard.putData("Auto Start Choices", m_chooser);
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
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_chooser.getSelected());
    System.out.println("autonomus choice:" + m_chooser.getSelected());

    // schedule the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    try {
      if (autoDone || m_autonomousCommand != null) //if we already finished this, or we are using a selected auto command, don't run this stuff
        return;

      Thread.sleep(1000);
      m_robotContainer.drive.drive(-0.3,0,0,false);

      if (m_chooser.getSelected() == "Simple Long") {
        Thread.sleep(6000);
      } else {
        Thread.sleep(3000);
      }
      m_robotContainer.drive.drive(0.0,0,0,false);

      CoralShootCommand csc = new CoralShootCommand(CoralShootCommand.CoralLevel.LEVEL1);
      csc.execute();
      Thread.sleep(500);
      csc.end(false);
      autoDone = true;
    } catch (Exception e) {}
  }

  @Override
  public void teleopInit() {
    //m_robotContainer.drive.resetOdometry(new Pose2d(0,0,new Rotation2d(Math.PI)));
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.)
    autoDone = false; //reset simple auto to run again the next match without recompile
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
