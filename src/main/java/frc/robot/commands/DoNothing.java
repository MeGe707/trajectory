package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DoNothing extends CommandBase {

    DriveSubsystem drivetrain;

    public DoNothing(DriveSubsystem drive) {
        drivetrain = drive;
        addRequirements(drivetrain);
    }

    public void initialize() {

    }

    @Override
    public void execute() {
        drivetrain.tankDriveVolts(0, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
