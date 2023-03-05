package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

public class MotorGroup {
    private TalonSRX masterTalonSRX;
    private TalonSRX followerTalonSRX;
    private Encoder encoder;
    private PIDController pidController;
    private double maxOutput;
    private double sensorPosition, output, error;
    private double kEncoderTickToFeet = 1/128*6*Math.PI/12;
    private double setPoint = 45;
    public String GroupName;

    public MotorGroup(int masterId, int followerId, int encoderId1, int encoderId2, double kP, double kI, double kD, double iLimit, double maxOutput, String name) {        
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
        driveMotors(-0.05);
    }
    /** Raises the robot based on human input on the controller.
    *
    * @return      void
    */
    public void raise_arm_manually() {
        driveMotors(0.05);
    }
    /** Raises the robot arm to the loading position
    *
    * @return      void
    */
    public void raise_arm_loading_position() {
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
        SmartDashboard.putNumber(GroupName + " Encoder Value" , encoder.get());
    }
}
