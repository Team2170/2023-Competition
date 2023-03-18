package frc.robot.subsystems.MotorGroups;

import javax.swing.GroupLayout.Group;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
    private CANSparkMax masterTalonSRX;
    private CANSparkMax followerTalonSRX;
    public RelativeEncoder encoder;
    public String GroupName;
    public int lowerBound;
    public int upperBound;
    private PIDController pidController;
    private double sensorPosition, output, error, iLimit;
    private double setPoint;

    private SparkMaxPIDController m_pidController;

    public ArmMotorGroup(int masterId, int followerId, int encoderIdA, int encoderIdB, PIDController pid, String name) {
        masterTalonSRX = new CANSparkMax(masterId,MotorType.kBrushed);
        followerTalonSRX = new CANSparkMax(followerId,MotorType.kBrushed);
        followerTalonSRX.follow(masterTalonSRX);
        /* Factory Default all hardware to prevent unexpected behaviour */
        masterTalonSRX.restoreFactoryDefaults();
        followerTalonSRX.restoreFactoryDefaults();
        followerTalonSRX.follow(masterTalonSRX);

        GroupName = name;
        setPoint = encoder.getPosition();

        encoder = masterTalonSRX.getAlternateEncoder(4096);
        m_pidController = masterTalonSRX.getPIDController();
    }

    public void displayEncoder() {
        double position = encoder.getPosition();
        double countsPerRev = encoder.getCountsPerRevolution();
        SmartDashboard.putNumber(GroupName + "Position", position);
        SmartDashboard.putNumber(GroupName + "Counts Per Revolution", countsPerRev);
    }

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

    public CANSparkMax GetMaster() {
        return this.masterTalonSRX;
    }

    public CANSparkMax GetFollower() {
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
        sensorPosition = getEncoderVal();
        //Get Error.
        error = setPoint - sensorPosition;  // difference between setpoint/reference and current point
        output = MathUtil.clamp(pidController.calculate(error), lowerBound, upperBound);
        return output;
    }

    public void setPoint(double point)
    {
        setPoint = point;
    }

    public double getEncoderVal()
    {
        return this.encoder.getPosition();
    }
}
