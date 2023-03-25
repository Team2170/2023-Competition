package frc.robot.subsystems.MotorGroups;

import com.ctre.phoenix.motorcontrol.ControlMode;
 
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class UpperArm extends ArmMotorGroup {
    public UpperArm(int masterId, int followerId, int encoderIdA, int encoderIdB, String name) {
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
        driveMotors(Constants.UpperArm.downwardspeed);
    };

    /**
     * Raises the robot based on human input on the controller.
     *
     * @return void
     */
    public void raise_arm_manually() {
        driveMotors(Constants.UpperArm.upwardspeed);
    };

    public void displayEncoder() {
        super.displayEncoder();
    }

    /**
     * Controls the motors.
     *
     * @return void
     */
    public void driveMotors(double speed) {
        super.GetMaster().set( speed);
        super.GetFollower().set(speed);
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
        return runOnce(() -> setGoal(Rotation2d.fromRadians(ArmConstants.UpperArm.groundDegrees)))
        .andThen(holdUntilSetpoint());
      }
    
      public Command low() {
        return runOnce(() -> setGoal(Rotation2d.fromRadians(ArmConstants.UpperArm.lowDegrees)))
        .andThen(holdUntilSetpoint());
      }
    
      public Command mid() {
        return runOnce(() -> setGoal(Rotation2d.fromRadians(ArmConstants.UpperArm.midDegrees)))
        .andThen(holdUntilSetpoint());
      }
    
      public Command travel() {
        return runOnce(() -> setGoal(Rotation2d.fromRadians(ArmConstants.UpperArm.travelDegrees)))
        .andThen(holdUntilSetpoint());
      }
}
