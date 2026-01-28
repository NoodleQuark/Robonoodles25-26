package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Constants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "KronkTeleOp", group = "a")
public class KronkTeleOp extends LinearOpMode {




    private DcMotor frontright;
    private DcMotor backright;
    private DcMotor backleft;
    private DcMotor frontleft;
    private DcMotor flywheelRight;
    private DcMotor flywheelLeft;
    private DcMotor intake;
    private DcMotor turret;
    private Servo indexRight;
    private Servo indexLeft;
    //This is the Gobuilda RGB Indicator it's just mapped as servo
    private Servo LED;
    private Servo LED2;
    private CRServo transferRight;
    private CRServo transferLeft;
    private ColorSensor colorSensor;
    private BHI260IMU imu;
    private SimplifiedOdometryRobot odometry;



    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // Put initialization blocks here.

        double magnitudeR;
        double Powervary = 1;
        double magnitudeL;
        double odoY;
        double odoX;
        int index = 0;
        long LTime = System.currentTimeMillis();
        boolean dpadPressed = false;
        boolean startPressed = false;
        boolean buttonPressed = false;
        boolean tank = true;
        boolean up = false;

        String lastColor = null;
        String[] balls = {"empty", "empty", "empty"};

        double targetHeading = 0;
        double kP = 3.5;


        backleft = hardwareMap.get(DcMotor.class, "leftback_drive");
        backright = hardwareMap.get(DcMotor.class, "rightback_drive");
        frontleft = hardwareMap.get(DcMotor.class, "leftfront_drive");
        frontright = hardwareMap.get(DcMotor.class, "rightfront_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        turret = hardwareMap.get(DcMotor.class, "turret");
        flywheelRight = hardwareMap.get(DcMotor.class, "flywheelRight");
        flywheelLeft = hardwareMap.get(DcMotor.class, "flywheelLeft");
        indexRight = hardwareMap.get(Servo.class, "indexRight");
        indexLeft = hardwareMap.get(Servo.class, "indexLeft");
        transferRight = hardwareMap.get(CRServo.class, "transferRight");
        transferLeft = hardwareMap.get(CRServo.class, "transferLeft");

        LED = hardwareMap.get(Servo.class, "LED");
        LED2 = hardwareMap.get(Servo.class, "LED2");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");


        odometry = new SimplifiedOdometryRobot(this);
        odometry.initialize(false);

        imu = hardwareMap.get(BHI260IMU.class, "imu");

        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)
        );
        imu.initialize(imuParams);

        /*imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);*/

        colorSensor.enableLed(true);
        Stopwatch timer = new Stopwatch();

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.

            Powervary = 1;
            while (opModeIsActive()) {

                //Sensors
                if(gamepad1.back){
                    //imu.initialize(parameters);
                    imu.resetYaw();
                    targetHeading= 0;
                    telemetry.addLine("IMU RESET");
                }

                if(gamepad1.start && !startPressed)tank = !tank;
                startPressed = gamepad1.start;

                telemetry.addLine(tank ? "TANK DRIVE" : "FIELD CENTRIC DRIVE");


                if(colorSensor.blue() > colorSensor.green()){
                    telemetry.addLine("purple");
                    lastColor = "purple";
                } else if (colorSensor.green() > 15+(colorSensor.red() + colorSensor.blue())/2) {
                    telemetry.addLine("green");
                    lastColor = "green";
                }else {
                    telemetry.addLine("no ball");
                }

                //Drivebase

                // ---------------------- DRIVE INPUT ----------------------
                double y = -gamepad1.right_stick_y; // forward/back
                double x = -gamepad1.right_stick_x; // strafe
                double rotationInput = gamepad1.left_stick_x; // manual rotation

                double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

// ---------------------- HEADING HOLD ----------------------
                double rotation;

                if (Math.abs(rotationInput) > 0.05) {
                    // Driver is rotating → update target heading to current heading
                    targetHeading = heading;
                    rotation = rotationInput;
                } else {
                    // No rotation input → hold heading using simple P control
                    double error = angleWrap(targetHeading - heading);
                    rotation = error * kP;
                }

// ---------------- FIELD CENTRIC TRANSFORM ----------------
                double tempX = x * Math.cos(-heading) - y * Math.sin(-heading);
                double tempY = x * Math.sin(-heading) + y * Math.cos(-heading);
                x = tempX;
                y = tempY;

// ----------------- MECANUM DRIVE -------------------------
                double frontLeftPower  = y + x + rotation;
                double backLeftPower   = y - x + rotation;
                double frontRightPower = y - x - rotation;
                double backRightPower  = y + x - rotation;

// ------------------ NORMALIZE ----------------------------
                double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
                        Math.max(Math.abs(backLeftPower),
                                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)))));

                frontLeftPower  /= max;
                backLeftPower   /= max;
                frontRightPower /= max;
                backRightPower  /= max;


                if (tank) {
                    telemetry.addLine("TANKY");
                    double rightStickX = -gamepad1.right_stick_x;
                    double rightStickY = gamepad1.right_stick_y;
                    double leftStickX  = -gamepad1.left_stick_x;
                    double leftStickY  = gamepad1.left_stick_y;

                    magnitudeR = Math.sqrt(Math.pow(rightStickX, 2) + Math.pow(rightStickY, 2));
                    magnitudeL = Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(leftStickY, 2));
                    if (magnitudeL > 1) magnitudeL = 1;
                    if (magnitudeR > 1) magnitudeR = 1;

                    if (rightStickX <= 1 && rightStickX >= 0 && rightStickY <= 0) {
                        frontright.setPower(Powervary * 1 * magnitudeR * (1 - 2 * rightStickX));
                        backright.setPower(Powervary * 1 * magnitudeR);
                    }
                    if (rightStickX >= -1 && rightStickX <= 0 && rightStickY >= 0) {
                        frontright.setPower(Powervary * -1 * magnitudeR * (1 + 2 * rightStickX));
                        backright.setPower(Powervary * -1 * magnitudeR);
                    }
                    if (rightStickX <= 1 && rightStickX >= 0 && rightStickY >= 0) {
                        frontright.setPower(Powervary * -1 * magnitudeR);
                        backright.setPower(Powervary * -1 * magnitudeR * (1 - 2 * rightStickX));
                    }
                    if (rightStickX >= -1 && rightStickX <= 0 && rightStickY <= 0) {
                        frontright.setPower(Powervary * 1 * magnitudeR);
                        backright.setPower(Powervary * 1 * magnitudeR * (1 + 2 * rightStickX));
                    }
                    if (leftStickX <= 1 && leftStickX >= 0 && leftStickY >= 0) {
                        frontleft.setPower(Powervary * -1 * magnitudeL * (1 - 2 * leftStickX));
                        backleft.setPower(Powervary * -1 * magnitudeL);
                    }
                    if (leftStickX >= -1 && leftStickX <= 0 && leftStickY <= 0) {
                        frontleft.setPower(Powervary * 1 * magnitudeL * (1 + 2 * leftStickX));
                        backleft.setPower(Powervary * 1 * magnitudeL);
                    }
                    if (leftStickX <= 1 && leftStickX >= 0 && leftStickY <= 0) {
                        frontleft.setPower(Powervary * 1 * magnitudeL);
                        backleft.setPower(Powervary * 1 * magnitudeL * (1 - 2 * leftStickX));
                    }
                    if (leftStickX >= -1 && leftStickX <= 0 && leftStickY >= 0) {
                        frontleft.setPower(Powervary * -1 * magnitudeL);
                        backleft.setPower(Powervary * -1 * magnitudeL * (1 + 2 * leftStickX));
                    }

                } else {
                    frontleft.setPower(frontLeftPower * Powervary);
                    backleft.setPower(backLeftPower * Powervary);
                    frontright.setPower(frontRightPower * Powervary);
                    backright.setPower(backRightPower * Powervary);
                }


                /*if(Math.abs(gamepad1.left_stick_y) != 0 || gamepad1.right_stick_button){
                    frontleft.setPower(-gamepad1.left_stick_y);
                    backleft.setPower(-gamepad1.left_stick_y);
                    frontright.setPower(-gamepad1.right_stick_y);
                    backright.setPower(-gamepad1.right_stick_y);
                }
                if(Math.abs(gamepad1.left_stick_x) != 0 && Math.abs(gamepad1.right_stick_y) == 0){
                    frontright.setPower(-gamepad1.left_stick_x);
                    backright.setPower(-gamepad1.left_stick_x);
                }*/


                if (gamepad1.a) {
                    Powervary = 0.25;
                }
                if (gamepad1.b) {
                    Powervary = 0.5;
                }
                if (gamepad1.x) {
                    Powervary = 0.75;
                }
                if (gamepad1.y) {
                    Powervary = 1;
                }

                //Outtake



                /*if(gamepad2.x)index=1;
                if(gamepad2.y)index=2;
                if (gamepad2.b)index=3;*/

                /*if(gamepad1.dpad_up){
                    frontleft.setPower(1);
                    frontright.setPower(1);
                    backright.setPower(1);
                    backleft.setPower(1);
                }*/

                //spin to next
                if(gamepad2.y && !buttonPressed){
                    buttonPressed = true;
                    index++;
                    if(index>2)index = 0;
                }

                //pressed after collection to spin to empty and store ball in data
                if (gamepad2.a && !buttonPressed) {
                    buttonPressed = true;
                    balls[index] = lastColor == null ? "?" : lastColor;
                    lastColor = null;

                    int next = (index + 1) % 3;
                    int next2 = (index + 2) % 3;

                    if (balls[next].equals("empty")) {
                        index = next;
                    } else if (balls[next2].equals("empty")) {
                        index = next2;
                    }
                }

                //spin to purple
                if(gamepad2.x && !buttonPressed){
                    buttonPressed = true;
                    for(int i = 0; i<3; i++){
                        if(!balls[i].equals("empty"))index = i;
                        if(balls[i].equals("purple")){
                            index = i;
                            break;
                        }
                    }
                }
                //spin to green
                if(gamepad2.b && !buttonPressed){
                    buttonPressed = true;
                    for(int i = 0; i<3; i++){
                        if(!balls[i].equals("empty"))index = i;
                        if(balls[i].equals("green")){
                            index = i;
                            break;
                        }
                    }
                }
                if (!gamepad2.a && !gamepad2.b && !gamepad2.x && !gamepad2.y) {
                    buttonPressed = false;
                }

                switch (balls[index]){
                    case "purple":
                        LED.setPosition(Constants.purple);
                        LED2.setPosition(Constants.purple);
                        break;
                    case "green":
                        LED.setPosition(Constants.green);
                        LED2.setPosition(Constants.green);
                        break;
                    case"empty":
                        LED.setPosition(Constants.white);
                        LED2.setPosition(Constants.white);
                        break;
                }

                /*if(!gamepad1.dpad_left && ! gamepad1.dpad_right)dpadPressed = false;
                if(gamepad1.dpad_right && !dpadPressed){
                    index++;
                    dpadPressed = true;
                }
                if(gamepad1.dpad_left && !dpadPressed){
                    dpadPressed = true;
                    index--;
                }
                if(index>3)index = 1;
                if(index<1)index=3;*/
                switch (index){
                    case 0:
                        indexLeft.setPosition(Constants.indexPosition1);
                        indexRight.setPosition(Constants.indexPosition1);
                        break;
                    case 1:
                        indexLeft.setPosition(Constants.indexPosition2);
                        indexRight.setPosition(Constants.indexPosition2);
                        break;
                    case 2:
                        indexLeft.setPosition(Constants.indexPosition3);
                        indexRight.setPosition(Constants.indexPosition3);
                        break;

                }
                telemetry.addData("indexRight", indexRight.getPosition());
                telemetry.addData("indexLeft", indexLeft.getPosition());

                //Rack and Pinions


                if(up && gamepad2.leftBumperWasPressed())timer.start();
                if(gamepad2.leftBumperWasReleased() && timer.running)timer.pause();
                if(gamepad2.leftBumperWasPressed() && timer.running)timer.resume();

                if(timer.running && timer.elapsed() >= 800){
                    balls[index] = "empty";
                    timer.end();
                }
                telemetry.addData("timer", timer.elapsed());

                if(gamepad2.right_bumper){
                    transferLeft.setPower(1);
                    transferRight.setPower(-1);
                } else if (gamepad2.left_bumper){
                    transferLeft.setPower(-1);
                    transferRight.setPower(1);
                }else{
                    transferLeft.setPower(0);
                    transferRight.setPower(0);
                }

                if(gamepad2.right_bumper && !up){
                    up = true;
                } else if (gamepad2.left_bumper && up) {
                    up = false;
                }





                //Turret
                turret.setPower(0.5*gamepad2.right_stick_x);

                //Flywheels
                if (!dpadPressed &&
                        ((Math.abs(flywheelLeft.getPower()+1) <= 0.05 && gamepad2.dpad_up) ||
                                (Math.abs(flywheelLeft.getPower()+Constants.flywheelClose) <= 0.05 && gamepad2.dpad_down) ||
                                (Math.abs(flywheelLeft.getPower()+Constants.flywheelMiddle) <= 0.05 && gamepad2.dpad_right) ||
                                (Math.abs(flywheelLeft.getPower()+Constants.flywheelMiddle) <= 0.05 && gamepad2.dpad_left))) {
                    flywheelLeft.setPower(0);
                    flywheelRight.setPower(0);
                } else if (gamepad2.dpad_down && !dpadPressed) {
                    flywheelLeft.setPower(Constants.flywheelClose);
                    flywheelRight.setPower(Constants.flywheelClose);
                } else if (gamepad2.dpad_right && !dpadPressed) {
                    flywheelLeft.setPower(Constants.flywheelMiddle);
                    flywheelRight.setPower(Constants.flywheelMiddle);
                } else if (gamepad2.dpad_left && !dpadPressed) {
                    flywheelLeft.setPower(Constants.flywheelMiddle);
                    flywheelRight.setPower(Constants.flywheelMiddle);
                } else if (gamepad2.dpad_up && !dpadPressed){
                    flywheelLeft.setPower(Constants.flywheelFar);
                    flywheelRight.setPower(Constants.flywheelFar);
                }
                if(gamepad2.right_trigger != 0){
                    flywheelRight.setPower(-gamepad2.right_trigger);
                    flywheelLeft.setPower(-gamepad2.right_trigger);
                } else if (gamepad2.left_trigger != 0) {
                    flywheelRight.setPower(0);
                    flywheelLeft.setPower(0);
                }
                if(!gamepad2.dpad_left && !gamepad2.dpad_up && !gamepad2.dpad_right && !gamepad2.dpad_down){
                    dpadPressed = false;
                }else{
                    dpadPressed = true;
                }
                telemetry.addData("dpadPressed", dpadPressed);

                telemetry.addData("flywheel power", flywheelLeft.getPower());

                //intake

                if(gamepad2.left_stick_y < 0){ // ball in
                    intake.setPower(-1);
                } else if(gamepad2.left_stick_y > 0){ // ball out
                    intake.setPower(1);
                } else if(gamepad2.left_stick_button){
                    intake.setPower(0);
                }

                // Telemetry

                telemetry.addData(" \n\n\n frontleft", frontleft.getPower());
                telemetry.addData("backleft", backleft.getPower());
                telemetry.addData("frontright", frontright.getPower());
                telemetry.addData("backright", backright.getPower());
                telemetry.addData("X (in)", "%.2f", odometry.strafeDistance);
                telemetry.addData("Y (in)", "%.2f", odometry.driveDistance);
                telemetry.addData("Heading (deg)", "%.2f", odometry.heading);
                telemetry.addData("Raw head", imu.getRobotYawPitchRollAngles());
                telemetry.addData("Comp head", heading);
                telemetry.addData("Time:", (System.currentTimeMillis()-LTime)/1000);
                telemetry.addData("Speed:", Powervary);
                telemetry.addLine("Joystick Tank Field Centric Drive \n Letter Buttons for speed limit \n Speed: " + Powervary);
                telemetry.addData("Index 1: ", balls[0]);
                telemetry.addData("Index 2: ", balls[1]);
                telemetry.addData("Index 3: ", balls[2]);


                telemetry.addData(" \n\n\n Clear", colorSensor.alpha());
                telemetry.addData("Red  ", colorSensor.red());
                telemetry.addData("Green", colorSensor.green());
                telemetry.addData("Blue ", colorSensor.blue());



                telemetry.update();
            }
            telemetry.update();
        }
    }
    private double angleWrap(double angle) {
        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }




}















public enum BallColor {
    PURPLE,
    GREEN,
    EMPTY,
    UNKNOWN
}

// Tracks the balls in the robot's 3-slot index
private BallColor[] balls = {BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY};
private int index = 0;                   // Current index position
private BallColor lastDetectedColor = BallColor.EMPTY;

// Button debounce flag
private boolean buttonPressed = false;

/**
 * Detects the color of the ball in the intake
 * Uses RGB ratios and brightness
 */
private BallColor detectBallColor(ColorSensor sensor) {
    int r = sensor.red();
    int g = sensor.green();
    int b = sensor.blue();

    // Minimum brightness check
    if (sensor.alpha() < 20) {
        return BallColor.EMPTY;
    }

    // Purple ball
    if (b > g * 1.15) {
        return BallColor.PURPLE;
    }

    // Green ball
    if (g > (r + b) / 2.0 + 10) {
        return BallColor.GREEN;
    }

    return BallColor.UNKNOWN;
}

// BALL STORAGE
/**
 * Store the last detected ball in the current index slot
 * then advance to the next empty slot
 */
private void storeBall() {
    balls[index] = lastDetectedColor;
    lastDetectedColor = BallColor.EMPTY;

    // Find next empty slot
    for (int i = 1; i <= balls.length; i++) {
        int next = (index + i) % balls.length;
        if (balls[next] == BallColor.EMPTY) {
            index = next;
            return;
        }
    }
}

// BALL SELECTION
/**
 * Select a ball of a specific color in the index
 * then cycle to that position.
 */
private void selectBall(BallColor target) {
    for (int i = 0; i < balls.length; i++) {
        if (balls[i] == target) {
            index = i;
            return;
        }
    }
}

// LEDS
/**
 * Updates the LED colors to display the color of the selected ball 
 */
private void updateLEDs(Servo LED, Servo LED2) {
    switch (balls[index]) {
        case PURPLE:
            LED.setPosition(Constants.purple);
            LED2.setPosition(Constants.purple);
            break;
        case GREEN:
            LED.setPosition(Constants.green);
            LED2.setPosition(Constants.green);
            break;
        default:
            LED.setPosition(Constants.white);
            LED2.setPosition(Constants.white);
            break;
    }
}












// ====== CLEAR SCORED BALL ======
private void clearBallAfterScore() {
    balls[index] = BallColor.EMPTY;
}

// ====== TELEOP LOOP EXAMPLE ======
public void ballControlLoop(ColorSensor colorSensor, Servo LED, Servo LED2) {
    // Detect ball color every loop
    lastDetectedColor = detectBallColor(colorSensor);

    // Store ball when driver presses a
    if (gamepad2.a && !buttonPressed) {
        buttonPressed = true;
        storeBall();
    }

    //Select purple ball for scoring when x pressed
    if (gamepad2.x && !buttonPressed) {
        buttonPressed = true;
        selectBall(BallColor.PURPLE);
    }

    //Select green ball for scoring when b pressed
    if (gamepad2.b && !buttonPressed) {
        buttonPressed = true;
        selectBall(BallColor.GREEN);
    }

    if (!gamepad2.a && !gamepad2.b && !gamepad2.x) buttonPressed = false;

    updateLEDs(LED, LED2);

    if (timer.running && timer.elapsed() >= 800) {
        clearBallAfterScore();
        timer.end();
    }
}
