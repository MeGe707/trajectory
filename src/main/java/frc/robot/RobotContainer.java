
package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import frc.robot.Constants.DriveTrainConstants;

import frc.robot.TrajectoriesContainer;
import frc.robot.TrajectoriesContainer.TrajectoryFilesAndPaths.BlueTrajectoryFilesAndPaths.BlueTrajectoryFilesAndPathsWithOutTurret;
import frc.robot.TrajectoriesContainer.TrajectoryFilesAndPaths.BlueTrajectoryFilesAndPaths.BlueTrajectoryFilesAndPathsWithTurret;
import frc.robot.TrajectoriesContainer.TrajectoryFilesAndPaths.RedTrajectoryFilesAndPaths.RedTrajectoryFilesAndPathsWithOutTurret;
import frc.robot.TrajectoriesContainer.TrajectoryFilesAndPaths.RedTrajectoryFilesAndPaths.RedTrajectoryFilesAndPathsWithTurret;

public class RobotContainer {

        final frc.robot.subsystems.DriveSubsystem m_robotDrive = new frc.robot.subsystems.DriveSubsystem();
        XboxController m_driverController = new XboxController(1);
        // trajectoryDefinitions

        public RobotContainer() {

                // Trajectory trajectoryBlue2BallLeftWithTurret =
                // TrajectoryUtil.fromPathweaverJson(
                // BlueTrajectoryFilesAndPathsWithTurret.trajectoryPathBlue2BallLeftWithTurret);
                // Trajectory trajectoryBlue2BallMidWithTurret =
                // TrajectoryUtil.fromPathweaverJson(
                // BlueTrajectoryFilesAndPathsWithTurret.trajectoryPathBlue2BallMidWithTurret);
                // Trajectory trajectoryBlue2BallRightWithTurret =
                // TrajectoryUtil.fromPathweaverJson(
                // BlueTrajectoryFilesAndPathsWithTurret.trajectoryPathBlue2BallRightWithTurret);

                // Trajectory trajectoryBlue2BallLeftWithOutTurret =
                // TrajectoryUtil.fromPathweaverJson(
                // BlueTrajectoryFilesAndPathsWithOutTurret.trajectoryPathBlue2BallLeftWithOutTurret);
                // Trajectory trajectoryBlue2BallMidWithOutTurret =
                // TrajectoryUtil.fromPathweaverJson(
                // BlueTrajectoryFilesAndPathsWithOutTurret.trajectoryPathBlue2BallMidWithOutTurret);
                // Trajectory trajectoryBlue2BallRightWithOutTurret =
                // TrajectoryUtil.fromPathweaverJson(
                // BlueTrajectoryFilesAndPathsWithOutTurret.trajectoryPathBlue2BallRightWithOutTurret);

                // Trajectory trajectoryRed2BallLeftWithTurret =
                // TrajectoryUtil.fromPathweaverJson(
                // RedTrajectoryFilesAndPathsWithTurret.trajectoryPathRed2BallLeftWithTurret);
                // Trajectory trajectoryRed2BallMidWithTurret =
                // TrajectoryUtil.fromPathweaverJson(
                // RedTrajectoryFilesAndPathsWithTurret.trajectoryPathRed2BallMidWithTurret);
                // Trajectory trajectoryRed2BallRightWithTurret =
                // TrajectoryUtil.fromPathweaverJson(
                // RedTrajectoryFilesAndPathsWithTurret.trajectoryPathRed2BallRightWithTurret);

                // Trajectory trajectoryRed2BallLeftWithOutTurret =
                // TrajectoryUtil.fromPathweaverJson(
                // RedTrajectoryFilesAndPathsWithOutTurret.trajectoryPathRed2BallLeftWithOutTurret);
                // Trajectory trajectoryRed2BallMidWithOutTurret =
                // TrajectoryUtil.fromPathweaverJson(
                // RedTrajectoryFilesAndPathsWithOutTurret.trajectoryPathRed2BallMidWithOutTurret);
                // Trajectory trajectoryRed2BallRightWithOutTurret =
                // TrajectoryUtil.fromPathweaverJson(
                // RedTrajectoryFilesAndPathsWithOutTurret.trajectoryPathRed2BallRightWithOutTurret);

                configureButtonBindings();

        }

        private void configureButtonBindings() {
                // Drive at half speed when the right bumper is held
                new JoystickButton(m_driverController, Button.kRightBumper.value)
                                .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
                                .whenReleased(() -> m_robotDrive.setMaxOutput(1));
        }

        public Command getAutonomousCommand() {

                return null;
        }
}
