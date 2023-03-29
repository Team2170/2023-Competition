// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;


  private static final String kDefaultAuto = "MidLane";
  private static final String kLeftLaneAuto = "LeftLane";
  private static final String kMidLaneAuto = "MidLane";
  private static final String kRightAuto = "RightLane";
  private static final String kForwardAuto = "Forward";
  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_led = new AddressableLED(8);
    m_ledBuffer = new AddressableLEDBuffer(10);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();


    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Forward", kForwardAuto);
    m_chooser.addOption("Left", kLeftLaneAuto);
    m_chooser.addOption("Mid", kMidLaneAuto);
    m_chooser.addOption("Right", kRightAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
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
    m_autoSelected = m_chooser.getSelected();
    m_robotContainer.setAutonomous(m_autoSelected);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if(m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }

    m_robotContainer.s_arm.periodic(0, 0,false,false);

    Timer.delay(0.1);   
}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_robotContainer.autoPeriodic();
    
  }

  @Override
  public void teleopInit() {
    m_autoSelected = m_chooser.getSelected();
    m_robotContainer.setAutonomous(m_autoSelected);

      // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.teleCreateCommand();
    m_robotContainer.s_Swerve.stop_drive();
    m_robotContainer.s_Swerve.hardReset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.periodic();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if(m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_robotContainer.autoPeriodic();
  }
}
