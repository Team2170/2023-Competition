package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units; 
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.subsystems.MotorGroups.Gains;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 9;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(21); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;


        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 20;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(118.74);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 14;
            public static final int angleMotorID = 15;
            public static final int canCoderID = 22;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(351.65);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 12;
            public static final int angleMotorID = 13;
            public static final int canCoderID = 21;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(122.16);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 16;
            public static final int angleMotorID = 17;
            public static final int canCoderID = 23;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(239);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class PnueMatics{
        public static final int forwardChannel = 11;
        public static final int reverseChannel = 10;
    }

    public static final class LowerArm{
        public static int MasterId = 32;
        public static int FollowerId = 31;
        public static int EncoderAId = 1;
        public static int EncoderBId = 0;
        public static double upwardspeed = 0.5;
        public static double downwardspeed = -0.8;
    }
    public static final class UpperArm{
        public static int MasterId = 30;
        public static int FollowerId = 33;
        public static int EncoderAId = 3;
        public static int EncoderBId = 2;
        public static double upwardspeed = 0.35;
        public static double downwardspeed = -0.4;
    }
    public static int ph = 30;


    public static final class Auton{
        public static final double balanceScale = 5.25, balanceScalePow = 1.0, balanceLimitDeg = 5.0;
    }

    public static final class AutonLeftLane
    {
        public static double forwardDistance = 14;
        public static double backwardDistance = 5;
        public static double strafeDistance = 5;
    }
    public static final class AutonMidLane
    {
        public static double forwardDistance = 14;
        public static double backwardDistance = 5;
    }
    public static final class AutonRightLane
    {
        public static double forwardDistance = 14;
        public static double backwardDistance = 5;
        public static double strafeDistance = 5;
    }

    public static final class ArmConstants {
        public static final class LowerArm {
            public static final int armPort = 30, dutyCyclePort = 0;
            public static final Gains armPosition = new Gains(0.85, 0, 0, 0, 0, 1.0);
            public static final double dutyCycleResolution = 1.0;
            public static final double absolutePositionOffset = 0.557;
            public static final double maxRadians = 1.5708;
            public static final double minRadians = 0.52;
            public static final double groundDegrees = 30.0;
            public static final double travelDegrees = 30.0;
            public static final double lowDegrees = 30.0;
            public static final double midDegrees = 90.0;
            public static final double toleranceRadians = 0.10;
            public static final double armInputScale = 2 * Math.PI / (maxRadians - minRadians);
            public static final double armOffset = minRadians + (maxRadians - minRadians) / 2;
            public static final double gravityFF = 0.05;
            public static final boolean encoderInverted = true;
        }
        public static final class UpperArm {
            public static final int armPort = 30, dutyCyclePort = 0;
            public static final Gains armPosition = new Gains(0.85, 0, 0, 0, 0, 1.0);
            public static final double dutyCycleResolution = 1.0;
            public static final double absolutePositionOffset = 0.557;
            public static final double maxRadians = 3.92699;
            public static final double minRadians = -0.52;
            public static final double groundDegrees = 30.0;
            public static final double travelDegrees = 30.0;
            public static final double lowDegrees = 30.0;
            public static final double midDegrees = 90.0;
            public static final double toleranceRadians = 0.10;
            public static final double armInputScale = 2 * Math.PI / (maxRadians - minRadians);
            public static final double armOffset = minRadians + (maxRadians - minRadians) / 2;
            public static final double gravityFF = 0.05;
            public static final boolean encoderInverted = true;
        }
    }
}
