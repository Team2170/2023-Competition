package frc.robot.subsystems;

import javax.swing.GroupLayout.Group;

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
    public double lastPos = 0.0;
    public double faked_lower_bound = 0.00;
    public double faked_upper_bound = 0.00;
    public boolean bounds_set = false;
    public Encoder encoder;

    public MotorGroup(int masterId, int followerId, int encoderId1,int encoderId2, double kP, double kI, double kD, double iLimit, double maxOutput, String name) {        
        this.maxOutput = maxOutput;
        encoder = new Encoder(encoderId1, encoderId2,true,EncodingType.k4X);
        masterTalonSRX = new TalonSRX(masterId);
        followerTalonSRX = new TalonSRX(followerId);
        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(iLimit);

        /* Factory Default all hardware to prevent unexpected behaviour */
        masterTalonSRX.configFactoryDefault();
        followerTalonSRX.configFactoryDefault();

        followerTalonSRX.follow(masterTalonSRX);
        encoder.reset();
        GroupName = name;
    }
    public void driveMotors(double speed) {
        masterTalonSRX.set(ControlMode.PercentOutput, speed);
    }

    public void operate()
    {
    }
    
    public void initialize_bounds()
    {
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
        driveMotors(-0.1);
    }

    public boolean allowArmMovement()
    {
        if(GroupName == "Upper")
        {
            return validate_upper();
        }
        if(GroupName == "Lower")
        {
            return validate_lower();
        }
        return false;
    }
    public boolean validate_lower()
    {
        if(encoder.get() <= Constants.LowerArm.lowerbound)
        {
            return false;
        }
        if(encoder.get() >= Constants.LowerArm.upperbound)
        {
            return false;
        }
        return true;
    }
    public boolean validate_upper()
    {
        if(encoder.get() <= faked_lower_bound)
        {
            return false;
        }
        if(encoder.get() >= faked_upper_bound)
        {
            return false;
        }
        return true;
    }
    /** Raises the robot based on human input on the controller.
    *
    * @return      void
    */
    public void raise_arm_manually() {
        driveMotors(0.4);
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

        // Output of encoder
        double output = encoder.get();

        // Output scaled by DistancePerPulse
        double distance = encoder.getDistance();

        //double raw = encoder.getRaw();

        SmartDashboard.putNumber(GroupName + "Output", output);
        SmartDashboard.putNumber(GroupName + "Distance", distance);
        //SmartDashboard.putNumber(GroupName + "Raw", raw);

        lastPos = output;
    }
}
