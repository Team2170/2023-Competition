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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

public class LeftLaneCommand extends AutoCommandBase {

    public LeftLaneCommand(Swerve s_Swerve, RobotArm s_Arm) {
        super(s_Swerve, s_Arm);
        super.s_Swerve = s_Swerve;
        super.s_Arm = s_Arm;
        super.step_zero = true;
        addRequirements(s_Swerve);
        addRequirements(s_Arm);
    }

    public void handle_auto_drive(Swerve s_Swerve) {
        if (super.step_zero) {
            // s_Arm.grabber.retract_piston();
            // Timer.delay(1);
            // s_Arm.grabber.extend_piston();
            // Timer.delay(1);
            // s_Arm.periodic(0, -0.5,false,false);
            // Timer.delay(0.25);   
            // s_Arm.periodic(0, 0,false,false);
            super.step_zero = false;
            super.step_one = true;
        } 
        else if (super.step_one) {
            super.step_one = false;
            super.step_two = true;
        } 
        else if (super.step_two) {
            if (checkDistance_x(s_Swerve, 14)) {
                System.out.println("REACHED SIDEWAYS STAGE");
                if (checkDistance_y(s_Swerve, 5)) {
                    super.step_two = false;
                    super.step_three = false;
                    super.step_four = true;
                } else {
                    drive_strafe_right(s_Swerve, 5);
                }            
            } else {
                drive_backward(s_Swerve, 14);
            }
        } 
        else if (super.step_three) {
            if (checkDistance_y(s_Swerve, 5)) {
                step_three = false;
                step_four = true;
            } else {
                drive_strafe_right(s_Swerve, 5);
            }
        }
        else if (super.step_four) {
            super.step_four = false;
            super.step_five = true;
        }
        else if (super.step_five) {
            if (checkDistance_x(s_Swerve, 5)) {
                super.step_zero = false;
                super.step_one = false;
                super.step_two = false;
                super.step_three = false;
                super.step_four = false;
                super.step_five = false;
            } else {
                drive_forward(s_Swerve, 5);
            }
        } else {
            double planeInclination = s_Swerve.getPlaneInclination().getDegrees();
            if(Math.abs(planeInclination) > Auton.balanceLimitDeg)
            {
                Translation2d balance = s_Swerve.getBalanceTranslation();
                Translation2d heading = new Translation2d(balance.getX(), 0);
                s_Swerve.drive(heading, 0, false, false);
            }  
        }
    }

    public CommandBase drive(Swerve s_Swerve) {
        Command instantForward = new InstantCommand(() -> handle_auto_drive(s_Swerve), s_Swerve);
        Command RepeatedForward = new RepeatCommand(instantForward);
        return Commands.sequence(RepeatedForward);
    }
}