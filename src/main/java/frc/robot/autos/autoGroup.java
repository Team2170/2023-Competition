package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AutoBalancer;
import frc.robot.subsystems.RobotArm;
import frc.robot.subsystems.Swerve;

public class autoGroup extends SequentialCommandGroup {
    /**
     * Creates a new ComplexAuto.
     *
     * @param s_Swerve The Swerve subsystem this command will run on
     * @param s_Arm The RobotArm subsystem this command will run on
     * @param s_balancer The AutoBalancer subsystem this command will run on
     */
    public autoGroup(Swerve s_Swerve,RobotArm s_Arm,AutoBalancer s_balancer ) {
      addCommands(
          // Drive forward the specified distance
          new driveOut(s_Swerve, s_Arm, s_balancer),
  
          // Drive to the charge station
          new driveToCharge(s_Swerve, s_Arm, s_balancer));
    }
  }
