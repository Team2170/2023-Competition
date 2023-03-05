package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

public class MotorGroup {
    private TalonSRX masterTalonSRX;
    private TalonSRX followerTalonSRX;
    //private Encoder encoder;
    private PIDController pidController;
    private double maxOutput;
    private double sensorPosition, output, error;
    private double kEncoderTickToFeet = 1/128*6*Math.PI/12;
    private double setPoint = 45;
    public String GroupName;
    public DutyCycleEncoder encoder;
    public double lastPos = 0.0;

    public MotorGroup(int masterId, int followerId, int encoderId1, double kP, double kI, double kD, double iLimit, double maxOutput, String name) {        
        this.maxOutput = maxOutput;
        encoder = new DutyCycleEncoder(encoderId1);
        masterTalonSRX = new TalonSRX(masterId);
        followerTalonSRX = new TalonSRX(followerId);
        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(iLimit);

        /* Factory Default all hardware to prevent unexpected behaviour */
        masterTalonSRX.configFactoryDefault();
        followerTalonSRX.configFactoryDefault();

        followerTalonSRX.follow(masterTalonSRX);
        // Set to 0.5 units per rotation
        encoder.setDistancePerRotation(0.05);
        GroupName = name;
    }
    public void driveMotors(double speed) {
        masterTalonSRX.set(ControlMode.PercentOutput, speed);
    }

    public void operate()
    {
        // Get Encoder Position
        sensorPosition = encoder.get() * kEncoderTickToFeet;
        //Get Error.
        error = setPoint - sensorPosition;  // difference between setpoint/reference and current point
        output = MathUtil.clamp(pidController.calculate(error), -maxOutput, maxOutput);
        System.out.println(output);
        driveMotors(output);
    }
    
    public void setTargetPoint(double value){
        setPoint = value;
    }
    /** Raises the robot based on human input on the controller.
    *
    * @return      void
    */
    public void stop_arm() {
        driveMotors(0);
    }

    /** Raises the robot based on human input on the controller.
    *
    * @return      void
    */
    public void lower_arm_manually() {
        if(allowArmMovement() == false)
        {
            stop_arm();
            return;
        }
        driveMotors(-0.05);
    }

    public boolean allowArmMovement()
    {
        if( GroupName == "Upper")
        {
            if( encoder.get() <= Constants.UpperArm.lowerbound)
            {
                return false;
            }
            if( encoder.get() >= Constants.UpperArm.upperbound)
            {
                return false;
            }
        }
        if( GroupName == "Lower")
        {
            if( encoder.get() <= Constants.LowerArm.lowerbound)
            {
                return false;
            }
            if( encoder.get() >= Constants.LowerArm.upperbound)
            {
                return false;
            }
        }
        return true;
    }

    /** Raises the robot based on human input on the controller.
    *
    * @return      void
    */
    public void raise_arm_manually() {

        if(allowArmMovement() == false)
        {
            stop_arm();
            return;
        }
        driveMotors(0.05);
    }
    /** Raises the robot arm to the loading position
    *
    * @return      void
    */
    public void raise_arm_loading_position() {
        if(allowArmMovement() == false)
        {
            stop_arm();
            return;
        }
        if(GroupName == "Upper")
        {
            setTargetPoint(Constants.UpperArm.armPositons.LOADING);
        }
        else
        {
            setTargetPoint(Constants.LowerArm.armPositons.LOADING);
        }        
    }
    /** Raises the robot arm to the mid scoring position
    *
    * @return      void
    */
    public void raise_arm_mid_scoring() {
        if(allowArmMovement() == false)
        {
            stop_arm();
            return;
        }
        if(GroupName == "Upper")
        {
            setTargetPoint(Constants.UpperArm.armPositons.MID);
        }
        else
        {
            setTargetPoint(Constants.LowerArm.armPositons.MID);
        }
    }
    /** Raises the robot arm to the mid scoring position
    *
    * @return      void
    */
    public void raise_arm_high_scoring() {
        if(GroupName == "Upper")
        {
            setTargetPoint(Constants.UpperArm.armPositons.HIGH);
        }
        else
        {
            setTargetPoint(Constants.LowerArm.armPositons.HIGH);
        }
    }
    /** Raises the robot arm to the low scoring position
    *
    * @return      void
    */
    public void raise_arm_low_scoring() {
        if(allowArmMovement() == false)
        {
            stop_arm();
            return;
        }
        if(GroupName == "Upper")
        {
            setTargetPoint(Constants.UpperArm.armPositons.LOW);
        }
        else
        {
            setTargetPoint(Constants.LowerArm.armPositons.LOW);
        }
    }



    public void run(){
        operate();
    }

    public void DisplayEncoder()
    {
        // Connected can be checked, and uses the frequency of the encoder
        boolean connected = encoder.isConnected();
        if (!connected)
        {
            output = lastPos;
            SmartDashboard.putNumber(GroupName + "Output", output);
            return;
        }

        // Duty Cycle Frequency in Hz
        int frequency = encoder.getFrequency();

        // Output of encoder
        double output = encoder.get();

        // Output scaled by DistancePerPulse
        double distance = encoder.getDistance();

        SmartDashboard.putBoolean(GroupName + "Connected", connected);
        SmartDashboard.putNumber(GroupName + "Frequency", frequency);
        SmartDashboard.putNumber(GroupName + "Output", output);
        SmartDashboard.putNumber(GroupName + "Distance", distance);

        lastPos = output;
    }
}
