package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowPathCommand extends SequentialCommandGroup {

        DriveSubsystem driveTrain;
        Trajectory trajectory;

        public FollowPathCommand(Path trajectoryPath1, DriveSubsystem driveTrain) throws IOException {

                Trajectory trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);

                RamseteCommand ramseteCommand = new RamseteCommand(
                                trajectory1,
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

                this.driveTrain = driveTrain;

                addCommands(
                                ramseteCommand,
                                new WaitCommand(2)

                );
        }

}
