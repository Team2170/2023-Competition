package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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
    private final XboxController operator = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton loadingButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton lowButton = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton midButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton highButton = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton leftTrigger = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton rightTrigger = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
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
                        () -> loadingButton.getAsBoolean(),
                        () -> lowButton.getAsBoolean(),
                        () -> midButton.getAsBoolean(),
                        () -> highButton.getAsBoolean(),
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
        s_arm.joint1.initialize_bounds();
        s_arm.joint2.initialize_bounds();

        /* Operator */
        boolean ManualMode = false;
        var manDir = this.operator.getLeftY();
        if (manDir > 0.2) {
            ManualMode = true;
        }        
        if (manDir < -0.2) {
            ManualMode = true;
        }
        if(ManualMode){
            System.out.println("Arm Movement Dir Manual Mode "+ manDir);
        }
        if(ManualMode)
        {
            s_arm.periodic(loadingButton.getAsBoolean(), lowButton.getAsBoolean(), midButton.getAsBoolean(), highButton.getAsBoolean(), ManualMode,
            manDir);
        }
        s_arm.DisplayEncoder();
    }

}
