package frc.robot.commands.Autos;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.Swerve;

public class AutoBalancePartial extends CommandBase {

    private Swerve s_Swerve;
    
    public AutoBalancePartial(Swerve swerve){
            this.s_Swerve = swerve; 
    }


    public void initialize() {
    }

    public void execute() {
        double planeInclination = s_Swerve.getPlaneInclination().getDegrees();
        if(Math.abs(planeInclination) > Auton.balanceLimitDeg)
        {
            Translation2d balance = s_Swerve.getBalanceTranslation();
            Translation2d heading = new Translation2d(balance.getX(), 0);
            s_Swerve.drive(heading, 0, false, false);
        }else{
            
            Translation2d heading = new Translation2d(0, 0);
            s_Swerve.drive(heading, 0, false, false);

        }
    }
    @Override
    public void end(boolean interrupted){
        s_Swerve.drive(new Translation2d(0, 0), 0, true, true);
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
