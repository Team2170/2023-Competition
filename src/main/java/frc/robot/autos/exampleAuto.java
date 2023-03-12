package frc.robot.autos;

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
import edu.wpi.first.wpilibj2.command.CommandBase;

public class exampleAuto extends CommandBase {
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

    public exampleAuto(Swerve s_Swerve, RobotArm arm,AutoBalancer s_Balancer) {
        this.s_Swerve = s_Swerve;
        this.s_Arm = arm;
        this.s_Balancer = s_Balancer;
        addRequirements(s_Swerve);
    }

    public void execute(double xVector, double yVector) {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(xVector, Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(yVector, Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(0, Constants.stickDeadband);
        double swerve_rotation = rotationVal * Constants.Swerve.maxAngularVelocity;
    }
}