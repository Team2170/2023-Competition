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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

public class MidLaneCommand extends CommandBase {
    private Swerve s_Swerve;
    private RobotArm s_Arm;
    private boolean step_one = false;
    private boolean step_two = false;
    private boolean step_three = false;
    private boolean step_four = false;
    private boolean step_zero = false;

    public MidLaneCommand(Swerve s_Swervel, RobotArm s_Arm) {
        this.s_Swerve = s_Swerve;
        this.s_Arm = s_Arm;
        this.step_zero = true;
        addRequirements(s_Swerve);
        addRequirements(s_Arm);
    }
    public boolean checkRotate(Swerve internalSwerve, double goalDistance) {
        double distance = internalSwerve.getYaw().getDegrees();
        System.out.println("Robot Distance Traveled " + distance);
        if (Math.abs(distance) >= goalDistance) {
            return false;
        }
        return true;
    }

    public boolean checkDistance_x(Swerve internalSwerve, double goalDistance) {
        double distance = internalSwerve.getPose().getX();
        System.out.println("Robot Distance Traveled " + distance);
        if (Math.abs(distance) >= goalDistance) {
            return false;
        }
        return true;
    }

    public boolean checkDistance_y(Swerve internalSwerve, double goalDistance) {
        double distance = internalSwerve.getPose().getY();
        System.out.println("Robot Distance Traveled " + distance);
        if (Math.abs(distance) >= goalDistance) {
            return false;
        }
        return true;
    }

    public void drive_forward(Swerve s_Swerve, double travelDistanceLimit) {
        boolean isDriving = checkDistance_x(s_Swerve, travelDistanceLimit * 0.3048);
        double driveDirection = 0.00;
        if (isDriving) {
            driveDirection = -1;
        }
        s_Swerve.drive(new Translation2d(driveDirection, 0), 0, true, true);
    }

    public void drive_backward(Swerve s_Swerve, double travelDistanceLimit) {
        boolean isDriving = checkDistance_x(s_Swerve, travelDistanceLimit * 0.3048);
        double driveDirection = 0.00;
        if (isDriving) {
            driveDirection = 1;
        }
        s_Swerve.drive(new Translation2d(driveDirection, 0), 0, true, true);
    }

    public void drive_strafe_left(Swerve s_Swerve, double travelDistanceLimit) {
        boolean isDriving = checkDistance_y(s_Swerve, travelDistanceLimit * 0.3048);
        double driveDirection = 0.00;
        if (isDriving) {
            driveDirection = 1;
        }
        s_Swerve.drive(new Translation2d(0, driveDirection), 0, true, true);
    }

    public void drive_strafe_right(Swerve s_Swerve, double travelDistanceLimit) {
        boolean isDriving = checkDistance_y(s_Swerve, travelDistanceLimit * 0.3048);
        double driveDirection = 0.00;
        if (isDriving) {
            driveDirection = -1;
        }
        s_Swerve.drive(new Translation2d(0, driveDirection), 0, true, true);
    }
    public void rotate_backward(Swerve s_Swerve, double travelDistanceLimit) {
        boolean isDriving = checkRotate(s_Swerve, travelDistanceLimit);
        double driveRotate = 0.00;
        if (isDriving) {
            driveRotate = 180;
        }
        s_Swerve.drive(new Translation2d(0, 0), driveRotate, true, true);
    }
    public void rotate_forward(Swerve s_Swerve, double travelDistanceLimit) {
        boolean isDriving = checkRotate(s_Swerve, travelDistanceLimit);
        double driveRotate = 0.00;
        if (isDriving) {
            driveRotate = 180;
        }
        s_Swerve.drive(new Translation2d(0, 0), driveRotate, true, true);
    }

    public void handle_auto_drive(Swerve s_Swerve) {
        if (step_zero) {
            if (checkRotate(s_Swerve, 0)) {
                step_zero = false;
                step_one = true;
            } else {
                rotate_forward(s_Swerve, 0);
            }
        } 
        if (step_one) {
            if (checkDistance_x(s_Swerve, 14)) {
                step_one = false;
                step_two = true;
            } else {
                drive_forward(s_Swerve, 14);
            }
        } else if (step_two) {
            step_two = false;
            step_three = true;
        } else if (step_three) {
            if (checkRotate(s_Swerve, -179)) {
                step_three = false;
                step_four = true;
            } else {
                rotate_backward(s_Swerve, -179);
            }
        }  else if (step_four) {
            if (checkDistance_x(s_Swerve, 5)) {
                step_zero = false;
                step_one = false;
                step_two = false;
                step_three = false;
                step_four = false;
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