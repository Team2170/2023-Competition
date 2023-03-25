package frc.robot.subsystems.MotorGroups;

import frc.robot.Constants;

public class LowerArm extends ArmMotorGroup {

    public double maxRangeOutput = 0; // TUNE THIS!
    public double minRangeOutput = 0; // TUNE THIS!

    /**
     * @param masterId
     * @param followerId
     * @param encoderIdA
     * @param encoderIdB
     * @param name
     */
    public LowerArm(int masterId, int followerId, int encoderIdA, int encoderIdB, String name) {
        super(masterId, followerId, name,Constants.LowerArm.gearDiameter);

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
     */
    public void driveMotors(double speed) {
        if (getPosition().getDegrees() >= maxRangeOutput) {
            return;
        }
        if (getPosition().getDegrees() >= maxRangeOutput) {
            return;
        }
        super.GetMaster().set(speed);
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
}
