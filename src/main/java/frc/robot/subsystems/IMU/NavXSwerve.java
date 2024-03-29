package frc.robot.subsystems.IMU;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

/**
 * Communicates with the NavX as the IMU.
 */
public class NavXSwerve extends SwerveIMU 
{

  /**
   * NavX IMU.
   */
  private AHRS   gyro;
  /**
   * Offset for the NavX yaw reading.
   */
  private double yawOffset = 0;

  /**
   * Constructor for the NavX swerve.
   *
   * @param port Serial Port to connect to.
   */
  public NavXSwerve(SerialPort.Port port)
  {
    try
    {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      gyro = new AHRS(port);
      SmartDashboard.putData(gyro);
    } catch (RuntimeException ex)
    {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }
  }

  /**
   * Reset IMU to factory default.
   */
  @Override
  public void factoryDefault()
  {
    // gyro.reset(); // Reported to be slow
    yawOffset = gyro.getYaw() % 360;
  }

  /**
   * Clear sticky faults on IMU.
   */
  @Override
  public void clearStickyFaults()
  {
  }

  /**
   * Set the yaw in degrees.
   *
   * @param yaw Yaw angle in degrees.
   */
  @Override
  public void setYaw(double yaw)
  {
    // gyro.reset(); // Reported to be slow using the offset.
    yawOffset = (yaw % 360) + (gyro.getYaw() % 360);
  }

  /**
   * Fetch the yaw/pitch/roll from the IMU.
   *
   * @param yprArray Array which will be filled with {yaw, pitch, roll} in degrees.
   */
  @Override
  public void getYawPitchRoll(double[] yprArray)
  {

    yprArray[0] = (gyro.getYaw() % 360) - yawOffset;
    yprArray[1] = (gyro.getPitch() % 360);
    yprArray[2] = (gyro.getRoll() % 360);
  }
  public double getYaw()
  {
    double yaw = (gyro.getYaw() % 360) - yawOffset;
    return yaw;
  }

  public double getPitch()
  {
    double pitch = (gyro.getPitch() % 360);
    return pitch;
  }


  public double getRoll()
  {
    double roll = (gyro.getRoll() % 360);
    return roll;
  }


  /**
   * Fetch the {@link Rotation3d} from the IMU. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  public Rotation3d getRotation3d()
  {
    return new Rotation3d(new Quaternion(gyro.getQuaternionW(),
                                         gyro.getQuaternionX(),
                                         gyro.getQuaternionY(),
                                         gyro.getQuaternionZ()))
        .minus(new Rotation3d(0, 0, Math.toRadians(yawOffset)));
  }

  /**
   * Fetch the acceleration [x, y, z] from the IMU in meters per second squared. If acceleration isn't supported returns
   * empty.
   *
   * @return {@link Translation3d} of the acceleration as an {@link Optional}.
   */
  @Override
  public Optional<Translation3d> getAccel()
  {
    return Optional.of(
        new Translation3d(
            gyro.getWorldLinearAccelX(),
            gyro.getWorldLinearAccelY(),
            gyro.getWorldLinearAccelZ())
            .times(9.81));
  }

  /**
   * Get the instantiated IMU object.
   *
   * @return IMU object.
   */
  @Override
  public Object getIMU()
  {
    return gyro;
  }
}