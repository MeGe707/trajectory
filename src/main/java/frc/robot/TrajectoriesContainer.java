package frc.robot;

import java.nio.file.Path;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public final class TrajectoriesContainer {

        public static final class TrajectoryFilesAndPaths {

                public static final class BlueTrajectoryFilesAndPaths {

                        public static final class BlueTrajectoryFilesAndPathsWithTurret {

                                public static final String trajectoryBlue2ballsLeftWithTurret1File = "paths/blue-2balls-left-withTurret1.wpilib.json";
                                public static final String trajectoryBlue2ballsMidWithTurret1File = "paths/blue-2balls-mid-withTurret1.wpilib.json";
                                public static final String trajectoryBlue2ballsRightWithTurret1File = "paths/blue-2balls-right-withTurret1.wpilib.json";

                                public static final String trajectoryBlue2ballsLeftWithTurret2File = "paths/blue-2balls-left-withTurret2.wpilib.json";
                                public static final String trajectoryBlue2ballsMidWithTurret2File = "paths/blue-2balls-mid-withTurret2.wpilib.json";
                                public static final String trajectoryBlue2ballsRightWithTurret2File = "paths/blue-2balls-right-withTurret2.wpilib.json";

                                public static final Path trajectoryPathBlue2ballsLeftWithTurret1 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryBlue2ballsLeftWithTurret1File);

                                public static final Path trajectoryPathBlue2ballsLeftWithTurret2 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryBlue2ballsLeftWithTurret2File);

                                public static final Path trajectoryPathBlue2ballsMidWithTurret1 = Filesystem
                                                .getDeployDirectory().toPath()
                                                .resolve(trajectoryBlue2ballsMidWithTurret1File);

                                public static final Path trajectoryPathBlue2ballsMidWithTurret2 = Filesystem
                                                .getDeployDirectory().toPath()
                                                .resolve(trajectoryBlue2ballsMidWithTurret2File);

                                public static final Path trajectoryPathBlue2ballsRightWithTurret1 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryBlue2ballsRightWithTurret1File);

                                public static final Path trajectoryPathBlue2ballsRightWithTurret2 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryBlue2ballsRightWithTurret2File);
                        }

                        public static final class BlueTrajectoryFilesAndPathsWithOutTurret {

                                public static final String trajectoryBlue2ballsLeftWithOutTurret1File = "paths/blue-2balls-left-withOutTurret1.wpilib.json";
                                public static final String trajectoryBlue2ballsMidWithOutTurret1File = "paths/blue-2balls-mid-withOutTurret1.wpilib.json";
                                public static final String trajectoryBlue2ballsRightWithOutTurret1File = "paths/blue-2balls-right-withOutTurret1.wpilib.json";

                                public static final String trajectoryBlue2ballsLeftWithOutTurret2File = "paths/blue-2balls-left-withOutTurret2.wpilib.json";
                                public static final String trajectoryBlue2ballsMidWithOutTurret2File = "paths/blue-2balls-mid-withOutTurret2.wpilib.json";
                                public static final String trajectoryBlue2ballsRightWithOutTurret2File = "paths/blue-2balls-right-withOutTurret2.wpilib.json";

                                public static final Path trajectoryPathBlue2ballsLeftWithOutTurret1 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryBlue2ballsLeftWithOutTurret1File);

                                public static final Path trajectoryPathBlue2ballsLeftWithOutTurret2 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryBlue2ballsLeftWithOutTurret2File);

                                public static final Path trajectoryPathBlue2ballsMidWithOutTurret1 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryBlue2ballsMidWithOutTurret1File);

                                public static final Path trajectoryPathBlue2ballsMidWithOutTurret2 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryBlue2ballsMidWithOutTurret2File);

                                public static final Path trajectoryPathBlue2ballsRightWithOutTurret1 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryBlue2ballsRightWithOutTurret1File);

                                public static final Path trajectoryPathBlue2ballsRightWithOutTurret2 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryBlue2ballsRightWithOutTurret2File);
                        }
                }

                public static final class RedTrajectoryFilesAndPaths {

                        public static final class RedTrajectoryFilesAndPathsWithTurret {

                                public static final String trajectoryRed2ballsLeftWithTurret1File = "paths/red-2balls-left-withTurret1.wpilib.json";
                                public static final String trajectoryRed2ballsMidWithTurret1File = "paths/red-2balls-mid-withTurret1.wpilib.json";
                                public static final String trajectoryRed2ballsRightWithTurret1File = "paths/red-2balls-right-withTurret1.wpilib.json";

                                public static final String trajectoryRed2ballsLeftWithTurret2File = "paths/red-2balls-left-withTurret2.wpilib.json";
                                public static final String trajectoryRed2ballsMidWithTurret2File = "paths/red-2balls-mid-withTurret2.wpilib.json";
                                public static final String trajectoryRed2ballsRightWithTurret2File = "paths/red-2balls-right-withTurret2.wpilib.json";

                                public static final Path trajectoryPathRed2ballsLeftWithTurret1 = Filesystem
                                                .getDeployDirectory().toPath()
                                                .resolve(trajectoryRed2ballsLeftWithTurret1File);

                                public static final Path trajectoryPathRed2ballsLeftWithTurret2 = Filesystem
                                                .getDeployDirectory().toPath()
                                                .resolve(trajectoryRed2ballsLeftWithTurret2File);

                                public static final Path trajectoryPathRed2ballsMidWithTurret1 = Filesystem
                                                .getDeployDirectory().toPath()
                                                .resolve(trajectoryRed2ballsMidWithTurret1File);

                                public static final Path trajectoryPathRed2ballsMidWithTurret2 = Filesystem
                                                .getDeployDirectory().toPath()
                                                .resolve(trajectoryRed2ballsMidWithTurret2File);

                                public static final Path trajectoryPathRed2ballsRightWithTurret1 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryRed2ballsRightWithTurret1File);

                                public static final Path trajectoryPathRed2ballsRightWithTurret2 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryRed2ballsRightWithTurret2File);

                        }

                        public static final class RedTrajectoryFilesAndPathsWithOutTurret {

                                public static final String trajectoryRed2ballsLeftWithOutTurret1File = "paths/red-2balls-left-withOutTurret1.wpilib.json";
                                public static final String trajectoryRed2ballsMidWithOutTurret1File = "paths/red-2balls-mid-withOutTurret1.wpilib.json";
                                public static final String trajectoryRed2ballsRightWithOutTurret1File = "paths/red-2balls-right-withOutTurret1.wpilib.json";

                                public static final String trajectoryRed2ballsLeftWithOutTurret2File = "paths/red-2balls-left-withOutTurret2.wpilib.json";
                                public static final String trajectoryRed2ballsMidWithOutTurret2File = "paths/red-2balls-mid-withOutTurret2.wpilib.json";
                                public static final String trajectoryRed2ballsRightWithOutTurret2File = "paths/red-2balls-right-withOutTurret2.wpilib.json";

                                public static final Path trajectoryPathRed2ballsLeftWithOutTurret1 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryRed2ballsLeftWithOutTurret1File);

                                public static final Path trajectoryPathRed2ballsLeftWithOutTurret2 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryRed2ballsLeftWithOutTurret2File);

                                public static final Path trajectoryPathRed2ballsMidWithOutTurret1 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryRed2ballsMidWithOutTurret1File);

                                public static final Path trajectoryPathRed2ballsMidWithOutTurret2 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryRed2ballsMidWithOutTurret2File);

                                public static final Path trajectoryPathRed2ballsRightWithOutTurret1 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryRed2ballsRightWithOutTurret1File);

                                public static final Path trajectoryPathRed2ballsRightWithOutTurret2 = Filesystem
                                                .getDeployDirectory()
                                                .toPath()
                                                .resolve(trajectoryRed2ballsRightWithOutTurret2File);

                        }

                }

        }

}
