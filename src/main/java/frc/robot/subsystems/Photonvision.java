package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Photonvision extends SubsystemBase{
    PhotonCamera cam;
    List<PhotonPipelineResult> unreadResults;
    PhotonPipelineResult result;
    PhotonTrackedTarget bestTarget;
    Boolean hasTarget;
    Double coralTagHeight = Units.inchesToMeters(8.75);
    Double cameraHeight = Units.inchesToMeters(5);
    Double cameraPitch = Units.degreesToRadians(0);
    Double goalDistance = 0.75; // meters
    SwerveSubsystem drivebase;

    SequentialCommandGroup aimAndRange;

    public Photonvision(SwerveSubsystem drivebase) {
        this.cam = new PhotonCamera("Camera_Module_v1");
        this.drivebase = drivebase;
        this.hasTarget = false;

        aimAndRange.addCommands(
            aimAtTarget(),
            approachTarget()
        );

        aimAndRange.addRequirements(this, drivebase);
        aimAndRange.andThen(NamedCommands.getCommand("test"), null);
    }

    public Command aimAtTarget(){
        return run(() -> {
            if(hasTarget){
                Double targetYaw = bestTarget.getYaw();
                drivebase.drive(
                    drivebase.getTargetSpeeds(
                        0,
                        0,
                        Rotation2d.fromDegrees(targetYaw)
                    )
                );
                System.out.println("target id: " + bestTarget.fiducialId);
            }
        });
    }

    public Command approachTarget(){
        return run(() -> {
            if(hasTarget){
                if(bestTarget.fiducialId == 11){
                    Transform3d targetRelativePosition = bestTarget.getBestCameraToTarget();
                    SequentialCommandGroup movementCommands = new SequentialCommandGroup(
                        drivebase.driveToDistanceHorizontalCommand(targetRelativePosition.getY(), 1), // left/right
                        drivebase.driveToDistanceCommand(targetRelativePosition.getX() - goalDistance, 1) // forward/back
                    );
                    movementCommands.execute();
                }
            }
        });
    }

    public Command fullVision(){
        return run(() -> {
            aimAndRange.execute();
        });
    }

    @Override
    public void periodic(){
        unreadResults = cam.getAllUnreadResults();
        if (!unreadResults.isEmpty()){
            result = unreadResults.get(unreadResults.size() - 1);
            if(result.hasTargets()){
                bestTarget = result.getBestTarget();
                hasTarget = true;
            }
            hasTarget = false;
        }
    }
}
