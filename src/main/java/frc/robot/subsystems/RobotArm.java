package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.MotorGroups.LowerArm;
import frc.robot.subsystems.MotorGroups.UpperArm;

public class RobotArm extends SubsystemBase {
    public UpperArm upper_arm;
    public LowerArm lower_arm;
    public PnuematicsGroup grabber;

    public RobotArm() {
        lower_arm = new LowerArm(Constants.LowerArm.MasterId,Constants.LowerArm.FollowerId,Constants.LowerArm.EncoderAId,Constants.LowerArm.EncoderBId,"Lower");
        upper_arm = new UpperArm(Constants.UpperArm.MasterId,Constants.UpperArm.FollowerId,Constants.UpperArm.EncoderAId,Constants.UpperArm.EncoderBId,"Upper");
        grabber = new PnuematicsGroup(Constants.ph);
    }

    public void periodic(double part_one_direction, double part_two_direction , boolean grab , boolean release) {
        lower_arm.operate_arm(part_one_direction);
        upper_arm.operate_arm(part_two_direction);
        grabber.operate(grab, release);
    }

    public void DisplayEncoder()
    {
        lower_arm.DisplayEncoder();
        upper_arm.DisplayEncoder();
    }
}