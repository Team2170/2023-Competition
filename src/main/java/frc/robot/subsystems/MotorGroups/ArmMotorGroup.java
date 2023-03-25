package frc.robot.subsystems.MotorGroups;

import javax.swing.GroupLayout.Group;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public abstract class ArmMotorGroup extends SubsystemBase {
    public DutyCycleEncoder absoluteArmEncoder;
    private ArmPIDController armPID;
    private Rotation2d localSetpoint;
    private CANSparkMax masterMotor;
    private CANSparkMax followerMotor;
    private SparkMaxAbsoluteEncoder absoluteEncoder;
    public String GroupName;
    public ArmMotorGroup(int masterId, int followerId, String name) {
        masterMotor = new CANSparkMax(masterId, MotorType.kBrushed);
        followerMotor = new CANSparkMax(followerId,MotorType.kBrushed);
        //masterMotor= new CANSparkMax(masterId,MotorType.kBrushed);
        //followerMotor = new CANSparkMax(followerId,MotorType.kBrushed);
        /* Factory Default all hardware to prevent unexpected behaviour */
        //masterMotor.restoreFactoryDefaults();
        //followerMotor.restoreFactoryDefaults();
        masterMotor.restoreFactoryDefaults();
        followerMotor.restoreFactoryDefaults();
        followerMotor.follow(masterMotor);

        absoluteEncoder = masterMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        // armPID =
        //     new ArmPIDController(
        //         ArmConstants.LowerArm.armPosition.P, ArmConstants.LowerArm.armPosition.I, ArmConstants.LowerArm.armPosition.D);
        // armPID.setAvoidanceRange(
        //     Rotation2d.fromRadians(ArmConstants.LowerArm.maxRadians),
        //     Rotation2d.fromRadians(ArmConstants.LowerArm.minRadians));
        // armPID.setTolerance(0.15);
        // setGoal(Rotation2d.fromRadians(ArmConstants.LowerArm.minRadians));
        // setDefaultCommand(hold());
        GroupName = name;

    }

    public void displayEncoder() {
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

    public CANSparkMax GetMaster()
    {
        return masterMotor;
    }
    public CANSparkMax GetFollower()
    {
        return followerMotor;
    }


  public void setMotor(double percent) {
    masterMotor.set(percent);
  }



  
  /**
   * Gets absolute position of the arm as a Rotation2d. Assumes the arm being level within frame is
   * the 0 point on the x axis. Assumes CCW+.
   *
   * @return current angle of arm
   */
  private Rotation2d getShiftedAbsoluteDistance() {
    var initialPosition =absoluteEncoder.getPosition() / ArmConstants.LowerArm.dutyCycleResolution;
    return Rotation2d.fromRotations(initialPosition)
        .minus(Rotation2d.fromRotations(ArmConstants.LowerArm.absolutePositionOffset));
  }

/**
   * Set arm with PID.
   *
   * @param setpoint setpoint in radians.
   */
  public void setGoal(Rotation2d setpoint) {
    localSetpoint = setpoint;
    // armPID.setSetpoint(setpoint);
    SmartDashboard.putNumber(GroupName + "Arm PID Setpoint", setpoint.getRadians());
  }
/**
   * Gets position of arm in radians. Assumes the arm being level within frame is the 0 point on the
   * x axis. Assumes CCW+.
   *
   * @return position in rad.
   */
  public Rotation2d getPosition() {
    return ArmConstants.LowerArm.encoderInverted
        ? getShiftedAbsoluteDistance().unaryMinus()
        : getShiftedAbsoluteDistance();
  }

  public void setArmHold() {
    var motorOutput =
        MathUtil.clamp(
            armPID.calculate(getPosition(), localSetpoint),
            -ArmConstants.LowerArm.armPosition.peakOutput,
            ArmConstants.LowerArm.armPosition.peakOutput);
    var feedforward = getPosition().getCos() * ArmConstants.LowerArm.gravityFF;

    setMotor(motorOutput + feedforward);

    SmartDashboard.putNumber(GroupName + "Arm PID Output", motorOutput);
    SmartDashboard.putNumber(GroupName + "Arm Feedforward", feedforward);
  }

  public Command hold() {
    return Commands.run(() -> setArmHold(), this);
  }
  public Command holdUntilSetpoint() {
    return hold().raceWith(Commands.waitSeconds(0.3).andThen(Commands.waitUntil(this::isAtSetpoint)));
  }

    /**
   * If the arm is at setpoint.
   *
   * @return if arm is at setpoint.
   */
  public boolean isAtSetpoint() {
    SmartDashboard.putBoolean(GroupName + "Arm PID at setpoint", armPID.atSetpoint());
    return armPID.atSetpoint();
  }

  @Override
  public void periodic() {
    // setArmHold();
    SmartDashboard.putNumber(GroupName + " Arm Raw Absolute Encoder", absoluteEncoder.getPosition());
    SmartDashboard.putNumber(GroupName + " Arm Processed Absolute Encoder", getPosition().getRadians());
    //SmartDashboard.putNumber(GroupName + " Arm PID error", armPID.getPositionError());
  }
}
