package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class NewAuto2Command extends CommandBase {

    private Swerve s_Swerve;
    
    public NewAuto2Command(Swerve swerve) {
        this.s_Swerve = swerve; 
    }

    public void initialize() {

    }

    public void execute() {
        return;
        // if(checkDistanceTraveled(s_Swerve, Constants.Auton.MidLane.forwardDistance, false )) {
        //     drive_backward(s_Swerve);
        // } 
    }
    public void drive_backward(Swerve s_Swerve) {
        s_Swerve.drive(new Translation2d(-1, 0), 0, true, true);
    }
    public void drive_forward(Swerve s_Swerve) {
        s_Swerve.drive(new Translation2d(1, 0), 0, true, true);
    }

    public boolean checkDistanceTraveled(Swerve internalSwerve, double goalDistance, boolean backwards) {
        double distance = internalSwerve.getDistanceTraveled();
        SmartDashboard.putNumber("Auto Distance Traveled " , distance);
        if( backwards )
        {
            if (Math.abs(distance) <= goalDistance) {
                return false;
            }
        }
        else{
            if (Math.abs(distance) >= goalDistance) {
                return false;
            }
        }

        return true;
    }
}
