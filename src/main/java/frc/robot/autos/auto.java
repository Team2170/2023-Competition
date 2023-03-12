package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.Swerve;

public class auto {
    /*
     * Drive Forward...
     */
    public void stage_one(Swerve s_Swerve)
    {
        s_Swerve.drive(new Translation2d(-1, 0), 0, false, false);
    }
    /*
     * Drive Backwards...
     */
    public void stage_two(Swerve s_Swerve)
    {
        s_Swerve.drive(new Translation2d(1, 0), 0, true, false);
    }
    /* 
     * AutoBalance Stage...
     */
    public void stage_three(Swerve s_Swerve)
    {

    }
}
