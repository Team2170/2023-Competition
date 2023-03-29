package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.RobotArm;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

public abstract class AutoCommandBase extends CommandBase {
    public Swerve s_Swerve;
    public RobotArm s_Arm;
    public boolean step_one = false;
    public boolean step_two = false;
    public boolean step_three = false;
    public boolean step_four = false;
    public boolean step_zero = false;
    public boolean step_five = false;

    public AutoCommandBase(Swerve s_Swerve, RobotArm s_Arm) {
        this.s_Swerve = s_Swerve;
        this.s_Arm = s_Arm;
        this.step_zero = true;
        addRequirements(s_Swerve);
        addRequirements(s_Arm);
    }
    
    public void drive_backward(Swerve s_Swerve) {
        s_Swerve.drive(new Translation2d(-1, 0), 0, true, true);
    }
    public void drive_forward(Swerve s_Swerve) {
        s_Swerve.drive(new Translation2d(1, 0), 0, true, true);
    }
    public void strafe_right(Swerve s_Swerve) {
        s_Swerve.drive(new Translation2d(0, 1), 0, true, true);
    }
    public void strafe_left(Swerve s_Swerve) {
        s_Swerve.drive(new Translation2d(0, -1), 0, true, true);
    }

    public boolean checkDistanceTraveled(Swerve internalSwerve, double goalDistance, boolean backwards) {
        double distance = internalSwerve.getDistanceTraveled();
        SmartDashboard.putNumber("Auto Distance Traveled " , distance);
        if( backwards )
        {
            if (Math.abs(distance) <= goalDistance) {
                return false;
            }
        }
        else{
            if (Math.abs(distance) >= goalDistance) {
                return false;
            }
        }

        
        return true;
    }

    public void lock_wheels(Swerve s_Swerve) {
        s_Swerve.lockPose();
    }

    public abstract CommandBase drive(Swerve s_Swerve, String name);
}