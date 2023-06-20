package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class DriveBackwardCommandPartial extends CommandBase {

    private Swerve s_Swerve;
    
    public DriveBackwardCommandPartial(Swerve swerve) {
        this.s_Swerve = swerve; 
    }

    public void initialize() {
    }

    @Override
    public void end(boolean interrupted){
        s_Swerve.drive(new Translation2d(0, 0), 0, true, true);
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
