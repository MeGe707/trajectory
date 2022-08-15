package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowPathCommand extends SequentialCommandGroup {

    DriveSubsystem driveTrain;
    Trajectory trajectory;

    RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
            driveTrain::getPose,
            new RamseteController(DriveTrainConstants.kRamseteB, DriveTrainConstants.kRamseteZeta),
            new SimpleMotorFeedforward(DriveTrainConstants.ksVolts,
                    DriveTrainConstants.kvVoltSecondsPerMeter,
                    DriveTrainConstants.kaVoltSecondsSquaredPerMeter),
            DriveTrainConstants.kDriveKinematics,
            driveTrain::getWheelSpeeds,
            new PIDController(DriveTrainConstants.kPDriveVel, 0, 0),
            new PIDController(DriveTrainConstants.kPDriveVel, 0, 0),
            driveTrain::tankDriveVolts,
            driveTrain);

    public FollowPathCommand(Trajectory trajectory, DriveSubsystem driveTrain) {

        this.driveTrain = driveTrain;

        addCommands(
                ramseteCommand,
                new WaitCommand(2)

        );
    }

}
