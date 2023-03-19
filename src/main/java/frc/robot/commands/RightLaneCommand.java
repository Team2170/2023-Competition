package frc.robot.commands;

import frc.robot.Constants;
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

public class RightLaneCommand extends AutoCommandBase {

    public RightLaneCommand(Swerve s_Swervel, RobotArm s_Arm) {
        super(s_Swervel, s_Arm);
        super.s_Swerve = s_Swerve;
        super.s_Arm = s_Arm;
        step_zero = true;
        addRequirements(s_Swerve);
        addRequirements(s_Arm);
    }


    public void handle_auto_drive(Swerve s_Swerve) {
        if (step_zero) {
            s_Arm.grabber.retract_piston();
            Timer.delay(1);
            s_Arm.grabber.extend_piston();
            Timer.delay(1);
            s_Arm.periodic(0, -0.5,false,false);
            Timer.delay(0.25);   
            s_Arm.periodic(0, 0,false,false);
            step_one = true;
        } 
        else if (step_one) {
            if (checkRotate(s_Swerve, 0)) {
                step_one = false;
                step_two = true;
            } else {
                rotate_forward(s_Swerve, 0);
            }
        } 
        else if (step_two) {
            if (checkDistance_x(s_Swerve, 14)) {
                step_two = false;
                step_three = true;
            } else {
                drive_forward(s_Swerve, 14);
            }
        } else if (step_three) {
            if (checkDistance_y(s_Swerve, 5)) {
                step_three = false;
                step_four = true;
            } else {
                drive_strafe_right(s_Swerve, 5);
            }
        } else if (step_four) {
            if (checkRotate(s_Swerve, -179)) {
                step_four = false;
                step_five = true;
            } else {
                rotate_backward(s_Swerve, -179);
            }
        }  else if (step_five) {
            if (checkDistance_x(s_Swerve, 5)) {
                step_zero = false;
                step_one = false;
                step_two = false;
                step_three = false;
                step_four = false;
                step_five = false;
            } else {
                drive_backward(s_Swerve, 5);
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