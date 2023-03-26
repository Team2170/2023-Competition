package frc.robot.subsystems.MotorGroups;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class LowerArm extends ArmMotorGroup {
    /**
     * @param masterId
     * @param followerId
     * @param name
     */
    public LowerArm(int masterId, int followerId, String name) {
        super(masterId, followerId, name,Constants.LowerArm.gearRatio, Constants.LowerArm.offset);

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
        driveMotors(Constants.LowerArm.downwardspeed);
    }

    /**
     * Raises the robot based on human input on the controller.
     *
     * @return void
     */
    public void raise_arm_manually() {
        driveMotors(Constants.LowerArm.upwardspeed);
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
        double arm_position = getPosition().getDegrees();
        if (arm_position >= Constants.LowerArm.minRangeOutput && arm_position <= Constants.LowerArm.maxRangeOutput ) {
            allow_drive = true;
        }
        SmartDashboard.putBoolean( GroupName + " Moving", allow_drive);
        if(allow_drive)
        {
            super.driveMotors(speed, Constants.LowerArm.maxRangeOutput, Constants.LowerArm.minRangeOutput);
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
