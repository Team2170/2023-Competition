package frc.robot.subsystems.MotorGroups;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ArmMotorGroup extends SubsystemBase {
    public DutyCycleEncoder absoluteArmEncoder;
    private CANSparkMax masterMotor;
    private CANSparkMax followerMotor;
    public String GroupName;

    private final AbsoluteEncoder m_turningEncoder;

    private double gearRadius;

    public ArmMotorGroup(int masterId, int followerId, String name, double gearDiameter) {
        masterMotor = new CANSparkMax(masterId, MotorType.kBrushed);
        followerMotor = new CANSparkMax(followerId,MotorType.kBrushed);
        masterMotor.restoreFactoryDefaults();
        followerMotor.restoreFactoryDefaults();
        followerMotor.follow(masterMotor);
        m_turningEncoder = masterMotor.getAbsoluteEncoder(Type.kDutyCycle);

        gearRadius = gearDiameter / 2 ;

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
   * Gets position of arm in degrees. Need to calculate what the position is in radians to be accurate .
   * how many rotations does the rev through bore turn to get the gear/sprocket one rotation?
   * The assumption is that the arm is parallel to the floor is the 0 position and the arm rotates positive in the CCW+ range.
   *
   * @return position in degrees.
   */
  public Rotation2d getPosition() {
    double radian = m_turningEncoder.getPosition() * Math.PI * gearRadius;
    double degrees = Math.toDegrees(radian);
    return new Rotation2d(degrees);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(GroupName + " Arm Raw Absolute Encoder", m_turningEncoder.getPosition());
    SmartDashboard.putNumber(GroupName + " Arm Position", getPosition().getDegrees());
  }
}
