package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@Autonomous(name = "RedAuto", group = "Auto")
public class RedAuto extends LinearOpMode {

    private DcMotor flywheelLeft;
    private DcMotor flywheelRight;
    private DcMotor intake;
    private Servo indexRight;
    private Servo indexLeft;
    private CRServo transferRight;
    private CRServo transferLeft;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private List<AprilTagDetection> detections;
    private int id = 0;
    private double index1 = 0.01;
    private double index2 = 0.365;
    private double index3 = 0.75;

    @Override
    public void runOpMode() {

        //Create odometry object with current opMode instance arg
        SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
        //call odometry init(see SimplifiedOdometryRobot method it's mostly just for encapsulation) with odometry telemetry displaying to control hub
        robot.initialize(true);


        flywheelLeft = hardwareMap.get(DcMotor.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotor.class, "flywheelRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        indexRight = hardwareMap.get(Servo.class, "indexRight");
        indexLeft = hardwareMap.get(Servo.class, "indexLeft");
        transferRight = hardwareMap.get(CRServo.class, "transferRight");
        transferLeft = hardwareMap.get(CRServo.class, "transferLeft");

        telemetry.addLine("Kronk Initialized");
        telemetry.addLine("\n Auto for Red Alliance audience side");
        telemetry.update();

        //Just in case
        robot.resetOdometry();
        robot.resetHeading();

        waitForStart();

        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();

        aprilTag.setDecimation(0);

        if (opModeIsActive()) {

            robot.drive(-50,0.5,0.11);
            camera(500);
            robot.turnTo(-15,0.25,0.25);
            camera(500);

            telemetry.addData("AprilTagID", id);
            telemetry.update();
            indexLeft.setPosition(index1);
            indexRight.setPosition(index1);
            robot.turnTo(30,0.5,0.25);
            robot.drive(10,0.5,0.25);
            sleep(200);
            //up
            transferLeft.setPower(1);
            transferRight.setPower(-1);
            sleep(1200);
            //down
            transferLeft.setPower(-1);
            transferRight.setPower(1);
            sleep(1200);
            transferLeft.setPower(0);
            transferRight.setPower(0);
            sleep(200);
            indexLeft.setPosition(index2);
            indexRight.setPosition(index2);
            sleep(300);
            //up
            transferLeft.setPower(1);
            transferRight.setPower(-1);
            sleep(1200);
            //down
            transferLeft.setPower(-1);
            transferRight.setPower(1);
            sleep(1200);
            transferLeft.setPower(0);
            transferRight.setPower(0);
            sleep(300);
            indexLeft.setPosition(index3);
            indexRight.setPosition(index3);
            sleep(300);
            //up
            transferLeft.setPower(1);
            transferRight.setPower(-1);
            sleep(1200);
            //down
            transferLeft.setPower(-1);
            transferRight.setPower(1);
            sleep(1200);
            transferLeft.setPower(0);
            transferRight.setPower(0);

            telemetry.update();

        }
    }
    private void camera(int duration){
        long starttime = System.currentTimeMillis();
        while(System.currentTimeMillis()-starttime < duration){
            detections = aprilTag.getDetections();
            if (detections.isEmpty()) {
                telemetry.addLine("No tags detected");
                break;
            } else {

                for (AprilTagDetection tag : detections) {
                    id = tag.id;
                }
                switch (id){
                    case 21:
                        index1 = 0.01;
                        index2 = 0.365;
                        index3 = 0.75;

                    case 22:
                        index1 = 0.75;
                        index2 = 0.01; // green
                        index3 = 0.365;

                    case 23:
                        index1 = 0.365;
                        index2 = 0.75;
                        index3 = 0.01;

                }
                break;
            }

        }
    }
}
