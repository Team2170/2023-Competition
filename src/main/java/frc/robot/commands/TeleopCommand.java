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
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopCommand extends CommandBase {
    private Swerve s_Swerve;
    private RobotArm s_Arm;
    private AutoBalancer s_Balancer;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier loading;
    private BooleanSupplier low;
    private BooleanSupplier mid;
    private BooleanSupplier high;
    private BooleanSupplier leftTrigger;
    private BooleanSupplier rightTrigger;

    public TeleopCommand(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, RobotArm arm,
            BooleanSupplier loading, BooleanSupplier low, BooleanSupplier mid, BooleanSupplier high,
            BooleanSupplier leftTrigger, BooleanSupplier rightTrigger, AutoBalancer s_Balancer) {
        this.s_Swerve = s_Swerve;
        this.s_Arm = arm;
        this.s_Balancer = s_Balancer;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.loading = loading;
        this.low = low;
        this.mid = mid;
        this.high = high;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
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
        /* Drive */
        if (balancing_mode) {
            this.s_Balancer.periodic(s_Swerve.gyro);
            translationVal = this.s_Balancer.translationVal;
            strafeVal = this.s_Balancer.strafeVal;
            Translation2d heading = new Translation2d(translationVal, strafeVal);
            s_Swerve.drive(
                    heading.times(Constants.Swerve.maxSpeed),
                    swerve_rotation,
                    !robotCentricSup.getAsBoolean(),
                    true);
        } else {
            s_Swerve.drive(
                    new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                    swerve_rotation,
                    !robotCentricSup.getAsBoolean(),
                    true);
        }

    }
}