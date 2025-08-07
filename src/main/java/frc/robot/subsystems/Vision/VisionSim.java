package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.Constants;

//import static frc.robot.Constants.FIELD_LAYOUT;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSim {

    private PhotonCameraSim camSim;
    private VisionSystemSim visSim;

   public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

    public VisionSim(PhotonCamera cam_in){
        if(Robot.isSimulation()){
            visSim = new VisionSystemSim("main");

          //  visSim.addAprilTags(FIELD_LAYOUT);

            var camProp = new SimCameraProperties();
            camProp.setCalibration(0, 0, Rotation2d.fromDegrees(70));
            camProp.setCalibError(0, 0);
            camProp.setFPS(70);
            camProp.setLatencyStdDevMs(0);

            camSim = new PhotonCameraSim(cam_in, camProp);

            visSim.addCamera(camSim,kRobotToCam);

            camSim.enableDrawWireframe(true);
        }
    }



    public void simulationPeriodic(Pose2d robotSimPose){
        visSim.update(robotSimPose);
    }

    public void resetSimPose(Pose2d pose){
        if(Robot.isSimulation()) visSim.resetRobotPose(pose);
    }


    public Field2d getSimDebugField(){
        if(!Robot.isSimulation()) return null;
        return visSim.getDebugField();
    }
}
