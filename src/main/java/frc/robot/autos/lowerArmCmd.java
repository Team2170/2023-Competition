package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.AutoBalancer;
import frc.robot.subsystems.RobotArm;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class lowerArmCmd extends CommandBase {
    private Swerve s_Swerve;
    private RobotArm s_Arm;
    private AutoBalancer s_Balancer;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier leftTrigger;
    private BooleanSupplier rightTrigger;
    private boolean set_wheelLock;
    private boolean isCmdFinished = false;

    public lowerArmCmd(Swerve s_Swerve, RobotArm arm,AutoBalancer s_Balancer) {
        this.s_Swerve = s_Swerve;
        this.s_Arm = arm;
        this.s_Balancer = s_Balancer;
        addRequirements(s_Arm);
    }

    public void initialize() {

    }

    public void execute() {
        s_Arm.periodic(0, -0.5,false,false);
        Timer.delay(0.25);
        s_Arm.periodic(0, 0,false,false);
        isCmdFinished = true;
    } 

    public boolean isFinished() {
        return isCmdFinished;
    }

    protected void end() {  
        s_Arm.periodic(0, 0,false,false);
    }

    protected void interrupted() {
        end();
    }
}