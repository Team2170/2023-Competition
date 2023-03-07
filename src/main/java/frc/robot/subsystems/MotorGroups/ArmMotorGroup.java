package frc.robot.subsystems.MotorGroups;

import javax.swing.GroupLayout.Group;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

public abstract class ArmMotorGroup {
    private TalonSRX masterTalonSRX;
    private TalonSRX followerTalonSRX;
    public Encoder encoder;
    public String GroupName;
    public int lowerBound;
    public int upperBound;
    private PIDController pidController;
    private double sensorPosition, output, error, iLimit;
    private double setPoint;

    public ArmMotorGroup(int masterId, int followerId, int encoderIdA, int encoderIdB, PIDController pid, String name) {
        encoder = new Encoder(encoderIdA, encoderIdB, true, EncodingType.k4X);
        masterTalonSRX = new TalonSRX(masterId);
        followerTalonSRX = new TalonSRX(followerId);

        /* Factory Default all hardware to prevent unexpected behaviour */
        masterTalonSRX.configFactoryDefault();
        followerTalonSRX.configFactoryDefault();
        followerTalonSRX.follow(masterTalonSRX);
        encoder.reset();
        GroupName = name;
        pidController = pid;
        pidController.setTolerance(iLimit);
        setPoint = encoder.get();
    }

    public void DisplayEncoder() {
        double output = encoder.get();
        double distance = encoder.getDistance();
        SmartDashboard.putNumber(GroupName + "Output", output);
        SmartDashboard.putNumber(GroupName + "Distance", distance);
    }

    public abstract void initialize_bounds(int Upper, int Lower);

    /**
     * Stops the movement of the robot arm.
     *
     * @return void
     */
    public abstract void stop_arm();

    /**
     * Lowers the robot based on human input on the controller.
     *
     * @return void
     */
    public abstract void lower_arm_manually();

    /**
     * Raises the robot based on human input on the controller.
     *
     * @return void
     */
    public abstract void raise_arm_manually();

    public abstract void driveMotors(double speed);

    public TalonSRX GetMaster() {
        return this.masterTalonSRX;
    }

    public TalonSRX GetFollower() {
        return this.followerTalonSRX;
    }

    public double GetLowerBound() {
        return lowerBound;
    }

    public void SetLowerBound(int bound) {
        this.lowerBound = bound;
    }

    public double GetUpperBound() {
        return lowerBound;
    }

    public void SetUpperBound(int bound) {
        this.upperBound = bound;
    }

    public abstract void operate_arm(double manualDirection);

    public double operate()
    {
        // Get Encoder Position
        sensorPosition = encoder.get();
        //Get Error.
        error = setPoint - sensorPosition;  // difference between setpoint/reference and current point
        output = MathUtil.clamp(pidController.calculate(error), lowerBound, upperBound);
        return output;
    }

    public void setPoint(double point)
    {
        setPoint = point;
    }

    public int getEncoderVal()
    {
        return this.encoder.get();
    }
}
