package frc.robot.subsystems.IMU;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Communicates with the NavX as the IMU.
 */
public class NavXSwerve
{

  /**
   * NavX IMU.
   */
  private AHRS   gyro;
  /**
   * Offset for the NavX yaw reading.
   */
  private double yawOffset = 0;
  private double pitchOffset = 0;

  /**
   * Constructor for the NavX swerve.
   */
  public NavXSwerve()
  {
    try
    {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      gyro = new AHRS(SerialPort.Port.kUSB);
      SmartDashboard.putData(gyro);
    } catch (RuntimeException ex)
    {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }
  }

  /**
   * Reset IMU to factory default.
   */
  public void factoryDefault()
  {
    // gyro.reset(); // Reported to be slow
    yawOffset = gyro.getYaw() % 360;
  }

  /**
   * Clear sticky faults on IMU.
   */
  public void clearStickyFaults()
  {
  }

  /**
   * Set the yaw in degrees.
   *
   * @param yaw Yaw angle in degrees.
   */
  public void setYaw(double yaw)
  {
    // gyro.reset(); // Reported to be slow using the offset.
    yawOffset = (yaw % 360) + (gyro.getYaw() % 360);
  }
 /**
   * Set the yaw in degrees.
   *
   * @param yaw Yaw angle in degrees.
   */
  public void setPitch(double pitch)
  {
    // gyro.reset(); // Reported to be slow using the offset.
    pitchOffset = (pitch % 360) + (gyro.getPitch() % 360);
  }



  /**
   * Fetch the yaw/pitch/roll from the IMU.
   *
   * @param yprArray Array which will be filled with {yaw, pitch, roll} in degrees.
   */
  public void getYawPitchRoll(double[] yprArray)
  {
    yprArray[0] = (gyro.getYaw() % 360) - yawOffset;
    yprArray[1] = gyro.getPitch() % 360;
    yprArray[2] = gyro.getRoll() % 360;
  }

  public double getYaw()
  {
    return (gyro.getYaw() % 360) - yawOffset;
  }
  public double getPitch()
  {
    return (gyro.getPitch() % 360) - pitchOffset;
  }


  /**
   * Get the instantiated IMU object.
   *
   * @return IMU object.
   */
  public Object getIMU()
  {
    return gyro;
  }
}