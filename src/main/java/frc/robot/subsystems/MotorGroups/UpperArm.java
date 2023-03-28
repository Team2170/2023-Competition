package frc.robot.subsystems.MotorGroups;

import frc.robot.Constants;

public class UpperArm extends ArmMotorGroup {
    public UpperArm(int masterId, int followerId, String name) {
        super(masterId, followerId, name, Constants.UpperArm.gearRatio, Constants.UpperArm.offset);
    }

    /**
     * Stops the movement of the robot arm.
     *
     * @return void
     */
    public void stop_arm() {
        driveMotors(0);
    }

    /**
     * Lowers the robot based on human input on the controller.
     *
     * @return void
     */
    public void lower_arm_manually() {
        driveMotors(Constants.UpperArm.downwardspeed);
    }

    /**
     * Raises the robot based on human input on the controller.
     *
     * @return void
     */
    public void raise_arm_manually() {
        driveMotors(Constants.UpperArm.upwardspeed);
    }

    public void displayEncoder() {
        super.displayEncoder();
    }

    /**
     * Controls the motors.
     *
     * @return void
     */
    public void driveMotors(double speed) {
        boolean allow_drive = false;
        double min = Constants.UpperArm.minRangeOutput;
        double max = Constants.UpperArm.maxRangeOutput;
        double arm_position = getPosition().getDegrees();
        if (speed == 0) {
            return;
        }
        if (arm_position >= min && arm_position <= max) {
            if (speed == 0) {
                allow_drive = false;
            } else {
                allow_drive = true;
            }
        } else if (arm_position <= min && speed > 0) {
            allow_drive = true;
        } else if (arm_position >= max && speed < 0) {
            allow_drive = true;
        } else {
            allow_drive = false;
        }
        if (allow_drive) {
            super.driveMotors(speed, max, min);
        }
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
