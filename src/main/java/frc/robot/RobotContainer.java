package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.exampleAuto;
import frc.robot.commands.TeleopCommand;
import frc.robot.subsystems.AutoBalancer;
import frc.robot.subsystems.RobotArm;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    /* Operator Controls */
    private final int ArmLowerDirection = XboxController.Axis.kLeftY.value;
    private final int ArmUpperDirection = XboxController.Axis.kRightY.value;
    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton lockButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kA.value);
 /* Operator Buttons */
    private final JoystickButton loadingButton = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton lowButton = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton midButton = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton highButton = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton leftTrigger = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton rightTrigger = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
   

  
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final RobotArm s_arm = new RobotArm();
    private final AutoBalancer s_Balancer = new AutoBalancer();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopCommand(
                        s_Swerve,
                        () -> driver.getRawAxis(translationAxis),
                        () -> driver.getRawAxis(strafeAxis),
                        () -> driver.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean(),
                        s_arm,
                        () -> leftTrigger.getAsBoolean(),
                        () -> rightTrigger.getAsBoolean(),
                         s_Balancer));

        // Configure the button bindings
        configureButtonBindings();

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        lockButton.whileTrue(new RepeatCommand(new InstantCommand(s_Swerve::lockPose, s_Swerve)));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }

    public void periodic()
    {
        double upper_part_manual_direction = MathUtil.applyDeadband(this.operator.getRawAxis(ArmUpperDirection), Constants.stickDeadband);
        double lower_part_manual_direction = MathUtil.applyDeadband(this.operator.getRawAxis(ArmLowerDirection), Constants.stickDeadband);
        var grab_button = leftTrigger.getAsBoolean();//rightTrigger.getAsBoolean();
        var release_button = rightTrigger.getAsBoolean();//leftTrigger.getAsBoolean();
        s_arm.periodic(lower_part_manual_direction, upper_part_manual_direction,grab_button,release_button);
        s_arm.DisplayEncoder();
        s_Swerve.lock_wheels = lockButton.getAsBoolean();
    }
    public void autoPeriodic()
    {
        double translationVal = 0.00;
        double strafeVal = 0.00;
        boolean lockWheels = this.s_Balancer.periodic(s_Swerve.gyro.getPitch());
        double rotationVal = MathUtil.applyDeadband(driver.getRawAxis(rotationAxis), Constants.stickDeadband);
        double swerve_rotation = rotationVal * Constants.Swerve.maxAngularVelocity;
        if (lockWheels)
        {
            s_Swerve.lockPose();
        }
        else{
            translationVal = this.s_Balancer.translationVal;
            strafeVal = this.s_Balancer.strafeVal;
            Translation2d heading = new Translation2d(translationVal, strafeVal);
            s_Swerve.drive(
                    heading.times(Constants.Swerve.maxSpeed),
                    swerve_rotation,
                    !robotCentric.getAsBoolean(),
                    true);
        }
    }
}
