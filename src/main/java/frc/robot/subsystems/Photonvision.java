// package frc.robot.subsystems;

// import java.util.List;
// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// public class Photonvision extends SubsystemBase{

//     // https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf
//     int[] reef = {
//         17,18,19,20,21,22, // blue
//         6,7,8,9,10,11 // red
//     };

//     Double goalDistance = 1.0; // meters

//     PhotonCamera cam;
//     String cam_name = "Camera_Module_v1";

//     PhotonTrackedTarget bestTarget;
//     Boolean hasTarget;

//     PIDController transController;
//     PIDController angularController;

//     SwerveSubsystem drivebase;

//     public Photonvision(SwerveSubsystem drivebase) {
//         this.cam = new PhotonCamera(cam_name);
//         this.drivebase = drivebase;
//         this.hasTarget = false;

//         this.transController = new PIDController(0, 0, 0);
//         this.angularController = new PIDController(0, 0, 0);
//     }

//     @Override
//     public void periodic(){
//         hasTarget = updateTarget();
//     }

//     public boolean updateTarget(){
//         List<PhotonPipelineResult> unreadResults = cam.getAllUnreadResults();
//         if (!unreadResults.isEmpty()){
//             PhotonPipelineResult result = unreadResults.get(unreadResults.size() - 1);
//             if(result.hasTargets()){
//                 PhotonTrackedTarget bestTarget = result.getBestTarget();
//                 int id = bestTarget.fiducialId;
//                 if(contains(reef,id)){
//                     return true;
//                 }
//             }
//         }
//         return false;
//     }

//     public static boolean contains(int[] array, int value) {
//         for (int num : array) {
//             if (num == value) {
//                 return true;
//             }
//         }
//         return false;
//     }
// }
