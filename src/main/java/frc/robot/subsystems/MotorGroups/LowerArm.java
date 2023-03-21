package frc.robot.subsystems.MotorGroups;

import com.ctre.phoenix.motorcontrol.ControlMode;
 
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class LowerArm extends ArmMotorGroup {


    /**
     * @param masterId
     * @param followerId
     * @param encoderIdA
     * @param encoderIdB
     * @param name
     */
    public LowerArm(int masterId, int followerId, int encoderIdA, int encoderIdB, String name) {
        super(masterId, followerId, name);  

        // absoluteArmEncoder = new DutyCycleEncoder(encoderIdA);
        // double diameter = 2;
        // double distancePerRotation = Math.PI * diameter;
        // absoluteArmEncoder.setDistancePerRotation(distancePerRotation);    
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

    public void displayEncoder() {
        super.displayEncoder();
    }

    /**
     * Controls the motors.
     *
     * @return void
     * We discovered one of the motors is working against the other.
     * This should map one inverted.
     */
    public void driveMotors(double speed) {
        super.GetMaster().set(speed);
        super.GetFollower().set(-speed);
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

    public Command ground() {
        return runOnce(() -> setGoal(Rotation2d.fromRadians(ArmConstants.LowerArm.groundDegrees)))
        .andThen(holdUntilSetpoint());
      }
    
      public Command low() {
        return runOnce(() -> setGoal(Rotation2d.fromRadians(ArmConstants.LowerArm.lowDegrees)))
        .andThen(holdUntilSetpoint());
      }
    
      public Command mid() {
        return runOnce(() -> setGoal(Rotation2d.fromRadians(ArmConstants.LowerArm.midDegrees)))
        .andThen(holdUntilSetpoint());
      }
    
      public Command travel() {
        return runOnce(() -> setGoal(Rotation2d.fromRadians(ArmConstants.LowerArm.travelDegrees)))
        .andThen(holdUntilSetpoint());
      }
}
