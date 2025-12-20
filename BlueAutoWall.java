/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;


@Autonomous(name="BlueAutoWall", group="Auto")
public class BlueAutoWall extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor flywheelRight;
    private DcMotor flywheelLeft;
    private DcMotor intake;
    private DcMotor turret;
    private CRServo indexRight;
    private CRServo indexLeft;
    private ColorSensor colorSensor;
    private SimplifiedOdometryRobot odometry;

    private Limelight3A limelight;
    private IMU imu;
    private AnalogInput spindexerEncoder;

    private ElapsedTime     runtime = new ElapsedTime();

    private double index1 = 0.01;
    private double index2 = 0.365;
    private double index3 = 0.75;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to GoBuilda to determine the motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // this is random
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // just to get circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415); // don't change this
    static final double     DRIVE_SPEED             = 0.4; // 0.35

    private int id = 0;


    @Override
    public void runOpMode() {


        // Initialize the drive system variables.
        backLeft = hardwareMap.get(DcMotor.class, "leftback_drive");
        backRight = hardwareMap.get(DcMotor.class, "rightback_drive");
        frontLeft = hardwareMap.get(DcMotor.class, "leftfront_drive");
        frontRight = hardwareMap.get(DcMotor.class, "rightfront_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        turret = hardwareMap.get(DcMotor.class, "turret");
        flywheelRight = hardwareMap.get(DcMotor.class, "flywheelRight");
        flywheelLeft = hardwareMap.get(DcMotor.class, "flywheelLeft");
        indexRight = hardwareMap.get(CRServo.class, "indexRight");
        indexLeft = hardwareMap.get(CRServo.class, "indexLeft");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        spindexerEncoder = hardwareMap.get(AnalogInput.class, "spindexerEncoder");


        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)
        );
        imu.initialize(imuParams);
        imu.resetYaw();

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

// 22 SECONDS
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); // arpil tag #11?
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        limelight.start();

        // Wait for the game to start (driver presses START)
        waitForStart();

        //RUN CODE put the move stuff here
        flywheelLeft.setPower(Constants.flywheelAuto);
        flywheelRight.setPower(Constants.flywheelAuto);

        camera(1000);
        forwardIMU(0.35,60,2.5,10);
        telemetry.addData("AprilTagID", id);
        telemetry.update();
        sleep(1000);
        encoderDrive(0.5, -5, 5,-5,5,10);
        sleep(200);
        while (sort(index1)){}
        sleep(500);
        while (!shoot(index1+120)){}
        sleep(500);
        while (!shoot(index1+240)){}
        sleep(500);
        while (sort(index1+120)){}
        sleep(500);
        while (!shoot(index1+360)){} //edit
        sleep(500);
        while (sort(index1)){}


        //inside triangle
        //encoderDrive(0.8,14, -14,-14, 14, 10);

        //outside triangle
        encoderDrive(0.5,-7, 7,-7, 7, 10);
        encoderDrive(0.5,-15, -15,-15, -15, 10);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(100);  // pause to display final telemetry message.

    }

    private double angleWrap(double angle) {
        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    public void forwardIMU(double power, double inches, double kP, double timeoutS) {

        double targetHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        int flTarget = frontLeft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int frTarget = frontRight.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int blTarget = backLeft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int brTarget = backRight.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(flTarget);
        frontRight.setTargetPosition(frTarget);
        backLeft.setTargetPosition(blTarget);
        backRight.setTargetPosition(brTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                frontLeft.isBusy() && frontRight.isBusy() &&
                backLeft.isBusy() && backRight.isBusy()) {

            double heading =
                    -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double error = angleWrap(targetHeading - heading);
            double correction = error * kP;

            double fl = power + correction;
            double bl = power + correction;
            double fr = power - correction;
            double br = power - correction;

            double max = Math.max(1.0, Math.max(Math.abs(fl),
                    Math.max(Math.abs(bl),
                            Math.max(Math.abs(fr), Math.abs(br)))));

            fl /= max; bl /= max; fr /= max; br /= max;

            frontLeft.setPower(fl);
            backLeft.setPower(bl);
            frontRight.setPower(fr);
            backRight.setPower(br);
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // --- 6. Return to RUN_USING_ENCODER ---
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(150);
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double frontLeftInches, double frontRightInches,double backLeftInches, double backRightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

            }

            // Stop all motion;
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    private void camera(int duration){
        long starttime = System.currentTimeMillis();
        while(System.currentTimeMillis()-starttime < duration){
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
            LLResult llResult = limelight.getLatestResult();


            if (llResult != null && llResult.isValid()){
                Pose3D botPose = llResult.getBotpose_MT2();
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
                List<LLResultTypes.FiducialResult> detections = llResult.getFiducialResults();

                if (detections != null && !detections.isEmpty()) {
                    for (LLResultTypes.FiducialResult d : detections) {
                        telemetry.addData("Tag ID", d.getFiducialId());
                        id = d.getFiducialId();
                    }
                } else {
                    telemetry.addData("Tag ID", "None");
                }

                telemetry.update();
            }
            switch (id){
                case 21:
                    index1 = 0;
                    break;

                case 22:
                    index1 = 120;
                    break;

                case 23:
                    index1 = 240;
                    break;

            }

        }
    }

    public boolean sort(double targetAngle) {
        double current = getAngle();
        double error = (((current - targetAngle) + 360) % 360);
        /*telemetry.addData("current", current);
        telemetry.addData("tart", targetAngle);
        telemetry.addData("comp", (((current - targetAngle) + 360)));
        telemetry.addData("error", error);*/

        // Stop condition: if within 5 degrees
        if (error < 10 || current - targetAngle < 0) {
            indexLeft.setPower(0);
            indexRight.setPower(0);
            return true; // finished
        } else {
            // Move in negative direction only
            double power = Math.max(0.175, error / 360);
            indexLeft.setPower(-power);
            indexRight.setPower(-power);
            return false; // still moving
        }
    }


    public double getAngle() {
        double voltage = spindexerEncoder.getVoltage();
        return (voltage / 3.3) * 360.0;
    }


    public boolean shoot(double targetAngle) {
        double current = getAngle();
        double error = (targetAngle - current + 360) % 360;

        if(error < 10) {
            indexLeft.setPower(0);
            indexRight.setPower(0);
            return true;
        } else {
            double power = Math.max(0.175, error / 360);
            indexLeft.setPower(power);
            indexRight.setPower(power);
            return false;
        }
    }
}
