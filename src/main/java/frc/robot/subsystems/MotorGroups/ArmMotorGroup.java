package frc.robot.subsystems.MotorGroups;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public abstract class ArmMotorGroup extends SubsystemBase {
  public DutyCycleEncoder absoluteArmEncoder;
  private CANSparkMax masterMotor;
  private CANSparkMax followerMotor;
  public String GroupName;

  private final AbsoluteEncoder m_turningEncoder;
  // private final SparkMaxPIDController m_MotorPidController;
  private double gearRatio;
  // public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel,
  // minVel, maxAcc, allowedErr;
  private double armOffset;

  public ArmMotorGroup(int masterId, int followerId, String name, double gearDiameter, double offset) {
    masterMotor = new CANSparkMax(masterId, MotorType.kBrushed);
    followerMotor = new CANSparkMax(followerId, MotorType.kBrushed);
    masterMotor.restoreFactoryDefaults();
    followerMotor.restoreFactoryDefaults();
    followerMotor.follow(masterMotor);
    m_turningEncoder = masterMotor.getAbsoluteEncoder(Type.kDutyCycle);

    gearRatio = gearDiameter;

    GroupName = name;

    armOffset = offset;

    // m_MotorPidController = masterMotor.getPIDController();
    // m_MotorPidController.setFeedbackDevice(m_turningEncoder);

    // // PID coefficients
    // kP = 5e-5;
    // kI = 1e-6;
    // kD = 0;
    // kIz = 0;
    // kFF = 0.000156;
    // kMaxOutput = 1;
    // kMinOutput = -1;

    // // set PID coefficients
    // m_MotorPidController.setP(kP);
    // m_MotorPidController.setI(kI);
    // m_MotorPidController.setD(kD);
    // m_MotorPidController.setIZone(kIz);
    // m_MotorPidController.setFF(kFF);
    // m_MotorPidController.setOutputRange(kMinOutput, kMaxOutput);

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

  public void driveMotors(double speed, double max, double min) {
    boolean allow_drive = false;
    if (getPosition().getDegrees() >= max) {
      if (speed < 0) {
        allow_drive = true;
      }
      else{
        allow_drive = false;
      }
    }
    else if (getPosition().getDegrees() <= min) {
      if (speed > 0) {
        allow_drive = true;
      }
      else{
        allow_drive = false;
      }
    }
    else if(speed == 0)
    {
      allow_drive = false;
    }
    else{
      allow_drive = true;
    }
    SmartDashboard.putBoolean( GroupName + " Moving " , allow_drive);
    if (allow_drive) {
      masterMotor.set(speed);
      followerMotor.set(speed);
    }
    else{
      masterMotor.set(0);
      followerMotor.set(0);
    }

  }

  public CANSparkMax GetMaster() {
    return masterMotor;
  }

  public CANSparkMax GetFollower() {
    return followerMotor;
  }

  public void setMotor(double percent) {

    masterMotor.set(percent);
  }

  /**
   * Gets position of arm in degrees. Need to calculate what the position is in
   * radians to be accurate .
   * how many rotations does the rev through bore turn to get the gear/sprocket
   * one rotation?
   * The assumption is that the arm is parallel to the floor is the 0 position and
   * the arm rotates positive in the CCW+ range.
   *
   * @return position in degrees.
   */
  public Rotation2d getPosition() {
    double radian = m_turningEncoder.getPosition() * 2 * Math.PI / gearRatio;
    double modifiedReading = radian + Math.toRadians(armOffset); 
    return new Rotation2d(modifiedReading);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(GroupName + " Arm Raw Absolute Encoder", m_turningEncoder.getPosition());
    SmartDashboard.putNumber(GroupName + " Arm Position", getPosition().getDegrees());
  }
}
