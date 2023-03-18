package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.AutoBalancer;
import frc.robot.subsystems.RobotArm;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil; 
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

public class DriveForward extends CommandBase {
    private Swerve s_Swerve;

    public DriveForward(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(-1, Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(0, Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(0, Constants.stickDeadband);
        double swerve_rotation = rotationVal * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                swerve_rotation,
                false,
                true);
    }
    public static boolean checkDistance(Swerve internalSwerve,double goalDistance)
    {
        double distance = internalSwerve.getPose().getX();
        System.out.println("Robot Distance Traveled " + distance);
        if( Math.abs(distance) >= goalDistance )
        {
            return false;
        }
        return true;
    }

    public static void handle_auto_drive(Swerve s_Swerve)
    {
        boolean isDriving = checkDistance(s_Swerve,14 * 0.3048);
        double driveDirection = 0.00;
        if (isDriving) {
            driveDirection = -1;
        }
        s_Swerve.drive(new Translation2d(driveDirection, 0), 0, true, true);
    }

    public static CommandBase driveForward(Swerve s_Swerve) {
        Command instantForward = new InstantCommand(() -> handle_auto_drive(s_Swerve),s_Swerve);
        Command RepeatedForward = new RepeatCommand(instantForward);
        return Commands.sequence(RepeatedForward);
    }
}