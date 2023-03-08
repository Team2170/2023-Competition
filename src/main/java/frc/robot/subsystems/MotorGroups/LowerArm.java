package frc.robot.subsystems.MotorGroups;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class LowerArm extends ArmMotorGroup {
    /**
     * @param masterId
     * @param followerId
     * @param encoderIdA
     * @param encoderIdB
     * @param name
     */
    public LowerArm(int masterId, int followerId, int encoderIdA, int encoderIdB, String name , double kP , double kI , double kD) {
        super(masterId, followerId, encoderIdA, encoderIdB, new PIDController(kP, kI, kD) , name);  
    }

    /**
     * Stops the movement of the robot arm.
     *
     * @return void
     */
    public void stop_arm() {
        driveMotors(0);
    };

    /**
     * Lowers the robot based on human input on the controller.
     *
     * @return void
     */
    public void lower_arm_manually() {
        driveMotors(Constants.LowerArm.downwardspeed);
    };

    /**
     * Raises the robot based on human input on the controller.
     *
     * @return void
     */
    public void raise_arm_manually() {
        driveMotors(Constants.LowerArm.upwardspeed);
    };

    public void DisplayEncoder() {
        super.DisplayEncoder();
    }

    /**
     * Controls the motors.
     *
     * @return void
     * We discovered one of the motors is working against the other.
     * This should map one inverted.
     */
    public void driveMotors(double speed) {
        super.GetMaster().set(ControlMode.PercentOutput, speed);
        super.GetFollower().set(ControlMode.PercentOutput, -speed);
    }

    public void operate_arm(double manualDirection) {
        if (manualDirection > 0.2) {
            raise_arm_manually();
        } else if (manualDirection < -0.2) {
            lower_arm_manually();
        } else {
            stop_arm();
        }
    }
}
