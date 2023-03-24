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
public class NavXSwerve extends SwerveIMU {

  /**
   * NavX IMU.
   */
  private AHRS gyro;
  /**
   * Offset for the NavX yaw reading.
   */
  private double yawOffset = 0;
  private Rotation3d offset = new Rotation3d();

  /**
   * Constructor for the NavX swerve.
   *
   * @param port Serial Port to connect to.
   */
  public NavXSwerve(SerialPort.Port port) {
    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus. */
      /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
      /*
       * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
       * details.
       */
      gyro = new AHRS(port);
      SmartDashboard.putData(gyro);
      // SmartDashboard.putNumber("Gyro Yaw", gyro.getQuaternionY());
      // SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());
      // SmartDashboard.putNumber("Gyro Roll", gyro.getRoll());

    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }
  }

  /**
   * Reset IMU to factory default.
   */
  @Override
  public void factoryDefault() {
    // gyro.reset(); // Reported to be slow
    offset = new Rotation3d(
        new Quaternion(gyro.getQuaternionW(), gyro.getQuaternionX(), gyro.getQuaternionY(), gyro.getQuaternionZ()));
  }

  public void setOffset(Rotation3d offset)
  {
    this.offset = offset;
  }
  public void setYaw(double yaw)
  {

  }
  /**
   * Clear sticky faults on IMU.
   */
  @Override
  public void clearStickyFaults() {
  }

  public Rotation3d getRawRotation3d() {
    return new Rotation3d(
        new Quaternion(gyro.getQuaternionW(), gyro.getQuaternionX(), gyro.getQuaternionY(), gyro.getQuaternionZ()));
  }

  public Rotation3d getRotation3d() {
    return getRawRotation3d().minus(offset);
  }

  /**
   * Fetch the acceleration [x, y, z] from the IMU in meters per second squared.
   * If acceleration isn't supported returns
   * empty.
   *
   * @return {@link Translation3d} of the acceleration as an {@link Optional}.
   */
  @Override
  public Optional<Translation3d> getAccel() {
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
  public Object getIMU() {
    return gyro;
  }

  public void getYawPitchRoll(double[] yprArray)
  {

  }

  public double getYaw()
  {
    double yaw = 0.00;
    return yaw;
  }
  public double getRoll()
  {
    double yaw = 0.00;
    return yaw;
  }
  public double getPitch()
  {
    double yaw = 0.00;
    return yaw;
  }
}