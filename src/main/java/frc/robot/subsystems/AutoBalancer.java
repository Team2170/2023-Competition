package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
//Rizzshi
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.subsystems.IMU.NavXSwerve;

public class AutoBalancer extends SubsystemBase {
    /** Creates a new AutoBalance. */
    final PIDController balanceController;
    double setPointAngle, sensorAngle, error, errorRange, errorRate, errorSum, lastTimestamp, lastError, output;
    final double kP = 0.2, kI = 0.1, kD = 0.1, maxOutput = 1;
    public double translationVal = 0.00;
    public double strafeVal = 0.00;

    public AutoBalancer() {
        balanceController = new PIDController(kP, kI, kD);
        balanceController.setTolerance(2.0);
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public CommandBase exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    public void balance(NavXSwerve robotGyro) {
        // proportional term calculation
        setPointAngle = 0;

        error = setPointAngle - robotGyro.getPitch();

        output = MathUtil.clamp(balanceController.calculate(error), -maxOutput, maxOutput);

        System.out.println(error + "/" + output);
        
    }

    public boolean periodic(NavXSwerve robotGyro) {
        boolean lock = false;
        // This method will be called once per scheduler run
        if (Math.abs(sensorAngle) - setPointAngle > 2.5) {
            balance(robotGyro);
            if (sensorAngle < setPointAngle) {
                System.out.println("Driving forward");
                translationVal = 0.05;
                strafeVal = 0.00;
                lock = false;
            } else if (sensorAngle > setPointAngle) {
                System.out.println("Driving backwards");
                translationVal = -0.05;
                strafeVal = 0.00;
                lock = false;
            }
            else if (sensorAngle == setPointAngle) {
                System.out.println("Level");
                translationVal = 0.00;
                strafeVal = 0.00;
                lock = true;
            }
        }
        return lock;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
