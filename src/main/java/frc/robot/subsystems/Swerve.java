package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition; 
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.IMU.ADIS16448Swerve;
import frc.robot.subsystems.IMU.NavXSwerve;
import frc.robot.subsystems.IMU.SwerveIMU;

public class Swerve<SwerveIMU> extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;

    private final SwerveBalance swerveBalance;


    // public Pigeon2 gyro;
    public NavXSwerve gyro;
    //public NavXSwerve gyro;
    
    /**
     * Simulation of the swerve drive.
     */
    public boolean lock_wheels;
    /**
     * 
     */
    public Swerve() {
        lock_wheels = false;
        //gyro = new NavXSwerve(Port.kUSB);
        gyro = new NavXSwerve(Port.kUSB);
        gyro.factoryDefault();

        zeroGyro();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getRawYaw(), getModulePositions());
        swerveBalance = new SwerveBalance(Auton.balanceScale, Auton.balanceScalePow);
    
    }
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, double reductionFactor){
        Translation2d reducedTranslation = translation.times(reductionFactor);//new Translation2d(translation.getX() * reductionFactor,translation.getY() * reductionFactor); 
        double reducedRotation = rotation * reductionFactor;
        drive(reducedTranslation, reducedRotation, fieldRelative, isOpenLoop);
    }
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getRawYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }



    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    } 

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getRawYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setOffset(gyro.getRawRotation3d());      
    }



    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    /**
     * Point all modules toward the robot center, thus making the robot very
     * difficult to move. Forcing the robot to keep
     * the current pose.
     */
    public void lockPose() {       
        SmartDashboard.putBoolean("Mod 0 Locked", true);
        SmartDashboard.putBoolean("Mod 1 Locked", true);
        SmartDashboard.putBoolean("Mod 2 Locked", true);
        SmartDashboard.putBoolean("Mod 3 Locked", true);
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getRawYaw(), getModulePositions());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }


    /* HANDLES NEW AUTOBALANCER */
    
    
    /**
   * Gets the translation of the robot according to the swerve balance updater.
   *
   * @return translation of the robot.
   */
  public Translation2d getBalanceTranslation() {
    var gyroRot3d = gyro.getRotation3d();
    return swerveBalance.calculate(gyroRot3d).unaryMinus();
  }

    /**
   * Gets plane inclination with current robot plane and the plane z = 0.
   *
   * @return plane inclination in radians.
   */
  public Rotation2d getPlaneInclination() {
    return Rotation2d.fromRadians(
        Math.atan(Math.hypot(getRawPitch().getTan(), getRawRoll().getTan())));
  }

  public Rotation2d getRawYaw()
  {
    return Rotation2d.fromRadians(gyro.getRotation3d().getZ());
  }
  public double getYaw() {
      double yaw = getRawYaw().getDegrees();
      return yaw;
  }
  public Rotation2d getRawPitch()
  {
    return Rotation2d.fromRadians(gyro.getRotation3d().getY());
  }
  public double getPitch() {
    double yaw = getRawPitch().getDegrees();
    return yaw;
}  
public Rotation2d getRawRoll()
{
  return Rotation2d.fromRadians(gyro.getRotation3d().getX());
}
public double getRoll() {
    double yaw = Rotation2d.fromRadians(gyro.getRotation3d().getX()).getDegrees();
    return yaw;
}

}