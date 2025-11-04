package frc.robot.subsystems.swervedrive;

import org.photonvision.PhotonCamera;

public class ballDetection {
    private PhotonCamera camera;

    public ballDetection() {
        camera = new PhotonCamera("left_cam"); // subject to change
        camera.setPipelineIndex(0);
    }

    public void getTarget() {
        boolean targetVisible = false;
        double targetYaw = 0.0;
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    System.out.println(target);
                }
            }
        }
    }
}
