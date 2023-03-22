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
    
    public boolean checkRotate(Swerve internalSwerve, double goalDistance) {
        double distance = internalSwerve.getYaw().getDegrees();
        if (Math.abs(distance) >= goalDistance) {
            return false;
        }
        System.out.println("Robot Distance Traveled " + distance);
        return true;
    }

    public boolean checkDistance_x(Swerve internalSwerve, double goalDistance) {
        double distance = internalSwerve.getPose().getX();
        if (Math.abs(distance) >= goalDistance) {
            return true;
        }
        return false;
    }

    public boolean checkDistance_y(Swerve internalSwerve, double goalDistance) {
        double distance = internalSwerve.getPose().getY();
        System.out.println("Checking Distance Y " + distance );
        if (Math.abs(distance) >= goalDistance) {
            return true;
        }
        return false;
    }

    public void drive_forward(Swerve s_Swerve, double travelDistanceLimit) {
        boolean isDriving = checkDistance_x(s_Swerve, travelDistanceLimit * 0.3048);
        double driveDirection = 0.00;
        if (!isDriving) {
            driveDirection = -1;
        }
        System.out.println("Driving Forward " + isDriving + " drive distance " + travelDistanceLimit);
        s_Swerve.drive(new Translation2d(driveDirection, 0), 0, true, true);
    }

    public void drive_backward(Swerve s_Swerve, double travelDistanceLimit) {
        boolean isDriving = checkDistance_x(s_Swerve, travelDistanceLimit * 0.3048);
        double driveDirection = 0.00;
        if (!isDriving) {
            driveDirection = 1;
        }
        System.out.println("Driving Backward" + isDriving + " drive distance " + travelDistanceLimit);
        s_Swerve.drive(new Translation2d(driveDirection, 0), 0, true, true);
    }

    public void drive_strafe_left(Swerve s_Swerve, double travelDistanceLimit) {
        boolean isDriving = checkDistance_y(s_Swerve, travelDistanceLimit * 0.3048);
        double driveDirection = 0.00;
        if (!isDriving) {
            driveDirection = 1;
        }
        System.out.println("Driving Left " + isDriving + " drive distance " + travelDistanceLimit);
        s_Swerve.drive(new Translation2d(0, driveDirection), 0, true, true);
    }

    public void drive_strafe_right(Swerve s_Swerve, double travelDistanceLimit) {
        boolean isDriving = checkDistance_y(s_Swerve, travelDistanceLimit * 0.3048);
        double driveDirection = 0.00;
        if (!isDriving) {
            driveDirection = -1;
        }
        System.out.println("Driving Right " + isDriving + " drive distance " + travelDistanceLimit);
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

    public void lock_wheels(Swerve s_Swerve) {
        s_Swerve.lockPose();
    }

    public abstract CommandBase drive(Swerve s_Swerve);
}