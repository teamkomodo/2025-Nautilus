// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.util.BlinkinPattern;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import org.photonvision.PhotonCamera;
import static frc.robot.Constants.*;



public class RobotContainer {  
    private final Field2d field2d = new Field2d();

    //Inputs Devices
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT); 
    // private PhotonCamera camera;
    
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(field2d);
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();

    //private final double VISION_TURN_kP = 0.01;
    public RobotContainer() {
        configureBindings();
        registerNamedCommands();
    }    
    
    private void configureBindings() {

        //drivetrain
		Trigger leftBumperDriver = driverController.leftBumper();
		leftBumperDriver.onTrue(Commands.runOnce(() -> {drivetrainSubsystem.zeroGyro();}));

		// deadbands are applied in command
		drivetrainSubsystem.setDefaultCommand(drivetrainSubsystem.joystickDriveCommand(
				() -> -driverController.getLeftY(), // -Y (up) on joystick is +X (forward) on robot
				() -> -driverController.getLeftX(), // -X (left) on joystick is +Y (left) on robot
				() -> driverController.getRightX() // -X (left) on joystick is +Theta (counter-clockwise) on robot
		));


        Trigger driverControllerA = driverController.a();
        driverControllerA.whileTrue(drivetrainSubsystem.aimAndRangeCommand());



    //     boolean targetVisible = false;
    // double targetYaw = 0.0;
    // var results = camera.getAllUnreadResults();
    // if(!results.isEmpty()){
    //   var result = results.get(results.size() - 1);
    //   if(result.hasTargets()){
    //     for (var target : result.getTargets()){
    //       targetYaw = target.getYaw();
    //       targetVisible = true;
    //     }
    //   }
    // }
    // double turn = -1.0 * targetYaw * VISION_TURN_kP * MAX_ANGULAR_VELOCITY;

    // System.out.println(turn);

    }

    public void teleopInit() {
        Commands.runOnce(() -> {drivetrainSubsystem.zeroGyro();});
        Commands.runOnce(() -> ledSubsystem.setFramePatternCommand(BlinkinPattern.COLOR_1_AND_2_PATTERN_SPARKLE_COLOR_2_ON_COLOR_1));
    }
    
    public Command getAutonomousCommand() {
        return AutoBuilder.followPath(null);
    }

    private void registerNamedCommands() {

    }
}
