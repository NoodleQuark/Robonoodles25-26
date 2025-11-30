package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="AprilTag Test", group="Vision")
public class AprilTagTest extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private static final int DESIRED_TAG_ID = -1;
    int frames;

    @Override
    public void runOpMode() {

        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();

        aprilTag.setDecimation(0);


        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            frames++;
            telemetry.addData("Frames:", frames);

            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (detections.isEmpty()) {
                telemetry.addLine("No tags detected");
            } else {
                for (AprilTagDetection tag : detections) {
                    if (DESIRED_TAG_ID < 0 || tag.id == DESIRED_TAG_ID) {
                        telemetry.addData("Tag ID", tag.id);
                        telemetry.addData("X (m)", tag.ftcPose.x);
                        telemetry.addData("Y (m)", tag.ftcPose.y);
                        telemetry.addData("Yaw (deg)", tag.ftcPose.yaw);
                    }
                }
            }

            telemetry.update();
        }

        visionPortal.close();
    }
}
