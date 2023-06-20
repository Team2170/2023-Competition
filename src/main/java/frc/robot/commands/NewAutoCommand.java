package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class NewAutoCommand extends CommandBase {

    private Swerve s_Swerve;
    
    public NewAutoCommand(Swerve swerve) {
        this.s_Swerve = swerve; 
    }

    public void initialize() {
    }

    public void execute() {
        drive_backward(s_Swerve);
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
}
