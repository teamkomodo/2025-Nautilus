package frc.robot.subsystems.Sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import frc.robot.Robot;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import static frc.robot.Constants.*;


public class DrivetrainSim {

    PWMSim leftLeader;
    PWMSim rightLeader;


    //sim configs
    LinearSystem<N2, N2, N2> drivetrainSystem =
            LinearSystemId.identifyDrivetrainSystem(2.0, 0.5, 2.25, 0.3, DRIVETRAIN_WIDTH
);

    DifferentialDrivetrainSim drivetrainSimulator = 
            new DifferentialDrivetrainSim(
                drivetrainSystem,
                DCMotor.getFalcon500(2),
                8,
                DRIVETRAIN_WIDTH,
                WHEEL_DIAMETER / 2,
                null
        );


    //Sim Cam Configs
    //NEEDS TO MATCH IRL CAMERA SETTINGS

    double camDiagFOV = 100.0;
    double camPitch = 0;
    double camHeightOffGround = 0;
    double minTargetArea = 0.1;
    double maxLEDRange = 20;
    int CamResWidth = 640;
    int camResHeight = 480;
    PhotonCameraSim camSim;


    VisionSystemSim visSim = new VisionSystemSim("main");


    TargetModel targetModel = new TargetModel(
                    List.of(
                            new Translation3d(0, Units.inchesToMeters(-9.819867), Units.inchesToMeters(-8.5)),
                            new Translation3d(0, Units.inchesToMeters(9.819867), Units.inchesToMeters(-8.5)),
                            new Translation3d(0, Units.inchesToMeters(19.625), Units.inchesToMeters(8.5)),
                            new Translation3d(0, Units.inchesToMeters(-19.625), Units.inchesToMeters(8.5)))
        );

    double tgtXPos = Units.feetToMeters(54);
    double tgtYPos = Units.feetToMeters(27/2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
    Pose3d farTargetPose = new Pose3d(
                new Translation3d(tgtXPos, tgtYPos, 0),
                new Rotation3d(0.0, 0.0, Math.PI)
        );

    public DrivetrainSim(int leftMotor, int rightMotor, PhotonCamera camera){
                leftLeader = new PWMSim(leftMotor);
                rightLeader = new PWMSim(rightMotor);
        
                var visionTarget = new VisionTargetSim(farTargetPose, targetModel);
                visSim.addVisionTargets(visionTarget);
        }






        
    


   
}






