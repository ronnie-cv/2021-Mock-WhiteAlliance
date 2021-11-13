// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.PIDDriveForward;
import frc.robot.commands.PIDTurn;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousRoutine extends SequentialCommandGroup {
  /** Creates a new AutonomousRoutine. */
  public AutonomousRoutine(DriveTrain d) {
    DriveTrain driveTrain = d;
    String gameData;
gameData = DriverStation.getInstance().getGameSpecificMessage();
if(gameData.length() > 0)
{
  switch (gameData.charAt(0))
  {
    case 'B' :
      //Blue case code
      addCommands(new PIDDriveForward(driveTrain, 1.0, 0.3), new PIDTurn(driveTrain, 90.0, 0.3), new PIDDriveForward(driveTrain, 1.0, 0.3));
      break;
    case 'G' :
      //Green case code
      break;
    case 'R' :
      //Red case code
      break;
    case 'Y' :
      //Yellow case code
      break;
    default :
      //This is corrupt data
      break;
  }
} else {
  //Code for no data received yet
}
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand()); 
  }
}
