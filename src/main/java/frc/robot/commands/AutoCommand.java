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

public class AutoCommand extends AutoCommandBase {
    public boolean allow_auto;

    public String internal_name;
    public AutoCommand(Swerve s_Swerve, RobotArm s_Arm) {
        super(s_Swerve, s_Arm);
        super.s_Swerve = s_Swerve;
        super.s_Arm = s_Arm;
        super.step_zero = true;
        addRequirements(s_Swerve);
        addRequirements(s_Arm);
        allow_auto = true;
    }

    public void forward_only(Swerve s_Swerve)
    {
        midlane(s_Swerve);
    }
    public void midlane(Swerve s_Swerve)
    {
        Timer.delay(0.1);
        s_Arm.upper_arm.driveMotors(-0.2);
        s_Arm.lower_arm.driveMotors(-0.2);
        Timer.delay(0.5);
        s_Arm.grabber.retract_piston();
        Timer.delay(0.3);
        s_Arm.grabber.extend_piston();
        Timer.delay(0.3);   
        s_Arm.periodic(0, 0,false,false);
        double distance = Constants.Auton.MidLane.forwardDistance;
        // Drive Away
        do {
            drive_backward(s_Swerve);
            SmartDashboard.putBoolean("autobalance", allow_auto);
        } while (checkDistanceTraveled(s_Swerve, distance, false ));
        distance = Constants.Auton.MidLane.backwardDistance;
        // Drive toward
        do {
            drive_forward(s_Swerve);
            SmartDashboard.putBoolean("autobalance", allow_auto);
        } while (checkDistanceTraveled(s_Swerve, distance, true));
        // Strafe Right
        allow_auto = false;
    }
    public void leftlane(Swerve s_Swerve)
    {
        midlane(s_Swerve);
    }
    public void rightlane(Swerve s_Swerve)
    {
        midlane(s_Swerve);
    }





    public void handle_auto_drive(Swerve s_Swerve, String name) {
        
        if(name == "Forward")
        {
            forward_only(s_Swerve);
        }
        else if(name == "Left")
        {
            leftlane(s_Swerve);
        }
        else if(name == "Mid")
        {
            midlane(s_Swerve);
        }
        else if(name == "Right")
        {
            rightlane(s_Swerve);
        }
        else{
            midlane(s_Swerve);
        }

    }
    public void handle_auto_balance(Swerve s_Swerve) {
        double planeInclination = s_Swerve.getPlaneInclination().getDegrees();
        SmartDashboard.putNumber("gyroPitch", planeInclination);
        if (Math.abs(planeInclination) > Auton.balanceLimitDeg) {
            Translation2d balance = s_Swerve.getBalanceTranslation();
            Translation2d heading = new Translation2d(balance.getX(), 0);
            s_Swerve.drive(heading, 0, false, false,0.25);
        }
        else{
            s_Swerve.drive(new Translation2d(0,0), 0, false, false,0.25);
        }
    }

    public CommandBase drive(Swerve s_Swerve, String name) {
        handle_auto_drive(s_Swerve, name);
        Command instantForward;
        s_Swerve.stop_drive();
        instantForward = new InstantCommand(() -> handle_auto_balance(s_Swerve), s_Swerve);
        Command RepeatedForward = new RepeatCommand(instantForward);
        return Commands.sequence(RepeatedForward);
    }

}