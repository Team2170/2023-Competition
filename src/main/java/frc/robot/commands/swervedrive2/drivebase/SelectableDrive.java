// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive2.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.swervedrive2.drivebase.modes.Absolute;
import frc.robot.commands.swervedrive2.drivebase.modes.AbsoluteField;
import frc.robot.commands.swervedrive2.drivebase.modes.Teleop;
import frc.robot.subsystems.swervedrive2.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

import swervelib.SwerveController;

/**
 * An example command that uses an example subsystem.
 */
public class SelectableDrive extends CommandBase
{

  private SwerveSubsystem  swerve;

  /// MODES
  private Absolute AbsoluteHandler ;
  private AbsoluteField AbsoluteFieldHandler ;
  private Teleop TeleopHandler ;


  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public SelectableDrive(SwerveSubsystem swerve, Absolute internalAbsoluteHandler , AbsoluteField internalAbsoluteFieldHandler, Teleop internalTeleopHandler)
  {
    swerve = swerve;
    AbsoluteHandler = internalAbsoluteHandler;
    AbsoluteFieldHandler = internalAbsoluteFieldHandler;
    TeleopHandler = internalTeleopHandler;

    AbsoluteHandler.toggle_on();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    TeleopHandler.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    AbsoluteHandler.execute();
    AbsoluteFieldHandler.execute();
    TeleopHandler.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }

  public void enable_telop()
  {
    AbsoluteHandler.toggle_off();
    AbsoluteFieldHandler.toggle_off();
    TeleopHandler.toggle_on();
  }
  public void enable_absolute()
  {
    AbsoluteHandler.toggle_on();
    AbsoluteFieldHandler.toggle_off();
    TeleopHandler.toggle_off();
  }
  public void enable_absolutefield(){
    AbsoluteHandler.toggle_off();
    AbsoluteFieldHandler.toggle_on();
    TeleopHandler.toggle_off();
  }
}
