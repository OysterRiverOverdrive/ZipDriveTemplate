// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.plans;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.List;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class navxrun extends SequentialCommandGroup {
  /** Creates a new navxrun. */
  public navxrun(
    DrivetrainSubsystem drivetrain
  ) {

    AutoCreationCmd autodrive = new AutoCreationCmd();

    Command show =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(-.5, 1)),
            new Pose2d(-0.84,  1.30, new Rotation2d(-2 * Math.PI / 3)));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      show
    );
  }
}
