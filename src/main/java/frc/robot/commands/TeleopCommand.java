package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.RobotArm;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil; 
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopCommand extends CommandBase {
    private Swerve s_Swerve;
    private RobotArm s_Arm;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier leftTrigger;
    private BooleanSupplier rightTrigger;
    private boolean set_wheelLock; 
    private BooleanSupplier slowButton;

    private boolean isRunning;

    public TeleopCommand(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, RobotArm arm,
            BooleanSupplier leftTrigger, BooleanSupplier rightTrigger, BooleanSupplier slowButton) {
        this.s_Swerve = s_Swerve;
        this.s_Arm = arm;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
        this.slowButton = slowButton;
        this.isRunning = true;
    }

    
    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        double swerve_rotation = rotationVal * Constants.Swerve.maxAngularVelocity;
        boolean balancing_mode = false;
        if (this.leftTrigger.getAsBoolean()) {
            if (this.rightTrigger.getAsBoolean()) {
                balancing_mode = true;
            }
        }
        SmartDashboard.putBoolean("Mod 0 Locked", false);
        SmartDashboard.putBoolean("Mod 1 Locked", false);
        SmartDashboard.putBoolean("Mod 2 Locked", false);
        SmartDashboard.putBoolean("Mod 3 Locked", false);
        /* Drive */
        if (s_Swerve.lock_wheels)
        {
            s_Swerve.lockPose();
        }
        else {
            if (balancing_mode)
            {
                double planeInclination = s_Swerve.getPlaneInclination().getDegrees();
                if(Math.abs(planeInclination) > Auton.balanceLimitDeg)
                {
                    Translation2d balance = s_Swerve.getBalanceTranslation();
                    Translation2d heading = new Translation2d(balance.getX(), 0);
                    s_Swerve.drive(heading, 0, false, false);
                } 
            }else{
                if( s_Swerve.autoOrientEnabled)
                {
                    s_Swerve.autoOrient(translationVal, strafeVal);
                }
                else{
                    s_Swerve.drive(
                        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                        swerve_rotation,
                        !robotCentricSup.getAsBoolean(),
                        true);
                }

            }
        }
    }
}