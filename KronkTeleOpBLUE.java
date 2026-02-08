package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;


@TeleOp(name = "KronkTeleOpBLUE", group = "a")
public class KronkTeleOpBLUE extends LinearOpMode {



    double shootAngle;
    private DcMotor frontright;
    private DcMotor backright;
    private DcMotor backleft;
    private DcMotor frontleft;
    private DcMotor flywheelRight;
    private DcMotor flywheelLeft;
    private DcMotor intake;
    private DcMotor turret;
    private CRServo indexRight;
    private CRServo indexLeft;
    //This is the Gobuilda RGB Indicator it's just mapped as servo
    private Servo LED;
    private Servo LED2;
    private ColorSensor colorSensor;
    private ColorSensor leftColor;
    private ColorSensor rightColor;

    private BHI260IMU imu;
    private SimplifiedOdometryRobot odometry;
    private AnalogInput spindexerEncoder;
    private VoltageSensor voltageSensor;
    private double startAngle = 0;
    private long sortTime = 0;
    private int index = 0;


    GoBildaPrismDriver prism;

    PrismAnimations.Solid white = new PrismAnimations.Solid(Color.WHITE);
    PrismAnimations.Solid red = new PrismAnimations.Solid(Color.BLUE);
    PrismAnimations.Solid solid;
    PrismAnimations.Solid prevColor;



    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // Put initialization blocks here.

        double magnitudeR;
        double Powervary = 1;
        double magnitudeL;
        double currentAngle = 0;
        double flywheelPower = 0;
        double firePower = 0;
        long LTime = System.currentTimeMillis();
        long fireTime = 0;
        boolean dpadPressed = false;
        boolean startPressed = false;
        boolean buttonPressed = false;
        boolean rightStickButt = false;
        boolean tank = true;
        int prevIndex;
        int autoFireIndex = 0;
        int shootState = -1;
        boolean cycle = true;
        boolean case0 = false;
        boolean rightBumperPressed = true;
        boolean shooting = true;
        boolean aPressed = false;
        boolean xPressed = false;
        boolean bPressed = false;
        boolean yPressed = false;
        boolean leftBumperPressed = false;
        boolean firing = false;
        long firetime = 0;
        int shotDistance = 0;
        double time = 0;
        double startTime = System.currentTimeMillis();
        double flywheelOffset = 0;
        double flywheelOffset2 = 0;
        boolean leftStickButt = false;

        String lastColor = "empty";
        String[] sort = {"empty", "empty", "empty"};

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
        indexRight = hardwareMap.get(CRServo.class, "indexRight");
        indexLeft = hardwareMap.get(CRServo.class, "indexLeft");

        LED = hardwareMap.get(Servo.class, "LED");
        LED2 = hardwareMap.get(Servo.class, "LED2");
        prism = hardwareMap.get(GoBildaPrismDriver.class,"prism");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        leftColor = hardwareMap.get(ColorSensor.class, "leftColor");
        rightColor = hardwareMap.get(ColorSensor.class, "rightColor");
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        spindexerEncoder = hardwareMap.get(AnalogInput.class, "spindexerEncoder");



        prism.setStripLength(35);


        white.setBrightness(100);
        white.setStartIndex(0);
        white.setStopIndex(36);
        red.setBrightness(100);
        red.setStartIndex(0);
        red.setStopIndex(36);


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

                lastColor = sort[0];

                telemetry.addLine("1: ");
                if(colorSensor.blue() < 150 && colorSensor.green() < 150){
                    sort[0] = "empty";
                }else if(colorSensor.blue() > colorSensor.green()){
                    telemetry.addLine("purple");
                    sort[0] = "purple";
                } else if (colorSensor.green() > 15+(colorSensor.red() + colorSensor.blue())/2) {
                    telemetry.addLine("green");
                    sort[0] = "green";
                }

                telemetry.addLine("2: ");
                if(leftColor.blue() < 150 && leftColor.green() < 150){
                    sort[1] = "empty";
                }else
                if(leftColor.blue() > leftColor.green()){
                    telemetry.addLine("purple");
                    sort[1] = "purple";
                } else if (leftColor.green() > 15+(leftColor.red() + leftColor.blue())/2) {
                    telemetry.addLine("green");
                    sort[1] = "green";
                }

                telemetry.addLine("3: ");
                if(rightColor.blue() < 150 && rightColor.green() < 150){
                    sort[2] = "empty";
                }else
                if(rightColor.blue() > rightColor.green()){
                    telemetry.addLine("purple");
                    sort[2] = "purple";
                } else if (rightColor.green() > 15+(rightColor.red() + rightColor.blue())/2) {
                    telemetry.addLine("green");
                    sort[2] = "green";
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


                if (tank && !gamepad1.dpad_up) {
                    telemetry.addLine("TANKY");
                    double rightStickX = gamepad1.right_stick_x;
                    double rightStickY = gamepad1.right_stick_y;
                    double leftStickX  = gamepad1.left_stick_x;
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

                } else if(!gamepad1.dpad_up){
                    frontleft.setPower(frontLeftPower * Powervary);
                    backleft.setPower(backLeftPower * Powervary);
                    frontright.setPower(frontRightPower * Powervary);
                    backright.setPower(backRightPower * Powervary);
                }



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
                time = System.currentTimeMillis()-startTime;











                //Outtake


                if(gamepad1.dpad_up){
                    frontleft.setPower(1);
                    frontright.setPower(1);
                    backright.setPower(1);
                    backleft.setPower(1);
                }

                //spin to next
                prevIndex = index;

                if(gamepad2.y && !yPressed){
                    index--;
                    startAngle = getAngle();
                    if(index<0)index = 2;
                    shooting = true;
                    cycle = false;
                    sortTime = System.currentTimeMillis();
                }
                yPressed = gamepad2.y;

                if (gamepad2.a && !aPressed){
                    for (int i = 0; i<=2; i++){
                        if (sort[i].equals("empty")){
                            index = i;
                        }
                    }

                }
                aPressed = gamepad2.a;


                if (gamepad2.b && !bPressed){
                    for (int i = 0; i<=2; i++){
                        if (sort[i].equals("purple")){
                            index = i;
                        }
                    }

                }
                bPressed = gamepad2.b;

                if (gamepad2.x && !xPressed){
                    for (int i = 0; i<=2; i++){
                        if (sort[i].equals("green")){
                            index = i;
                        }
                    }
                }
                xPressed = gamepad2.x;

                if(!cycle && shooting){
                    switch (index){
                        case 0: cycle = sort(78); break;
                        case 1: cycle = sort(195); break;
                        case 2: cycle = sort(317); break;
                    }
                }






                //shoot
                if (gamepad2.right_bumper && !rightBumperPressed){
                    /*startAngle = getAngle();
                    cycle = true;
                    shooting = false;
                    index++;
                    if(index>2)index = 0;*/
                    if(firing){
                        firing = false;
                        indexLeft.setPower(0);
                        indexRight.setPower(0);
                        switch (shotDistance){
                            case 0:
                                flywheelPower = -0.4;
                                break;
                            case 1:
                                flywheelPower = -0.425;
                                break;
                            case 2:
                                flywheelPower = -0.5;
                                break;
                            case 3:
                                flywheelPower = -1;
                                break;
                        }

                    }else{
                        firing = true;
                        firetime = System.currentTimeMillis();
                        firePower = flywheelRight.getPower();
                        indexLeft.setPower(1);
                        indexRight.setPower(1);
                    }

                }

                if(firing){
                    switch(shotDistance){
                        case 0:
                            if(System.currentTimeMillis() - fireTime > 350){
                                flywheelPower = (firePower - 0.475);
                            }else if(System.currentTimeMillis() - fireTime > 50){
                                flywheelPower = (firePower - 0.1);
                            }
                            break;
                        case 1 :
                            if(System.currentTimeMillis() - fireTime > 350){
                                flywheelPower = (firePower - 0.575);
                            }else if(System.currentTimeMillis() - fireTime > 50){
                                flywheelPower = (firePower - 0.1);
                            }
                            break;
                        case 2 :
                            if(System.currentTimeMillis() - fireTime > 350){
                                flywheelPower = (firePower - 0.575);
                            }else if(System.currentTimeMillis() - fireTime > 50){
                                flywheelPower = (firePower - 0.125);
                            }
                            break;
                        case 3 :
                            if(System.currentTimeMillis() - fireTime > 350){
                                flywheelPower = (firePower - 0.575);
                            }else if(System.currentTimeMillis() - fireTime > 50){
                                flywheelPower = (firePower - 0.1);
                            }
                            break;
                    }

                }

                rightBumperPressed = gamepad2.right_bumper;

                if (gamepad2.left_bumper && !leftBumperPressed){
                    startAngle = getAngle();
                    cycle = true;
                    shooting = false;
                    index++;
                    if(index>2)index = 0;
                    sortTime = System.currentTimeMillis();
                }

                leftBumperPressed = gamepad2.left_bumper;

                if (!shooting && cycle){
                    switch (index){
                        case 0: shooting = shoot(120); break;
                        case 1: shooting = shoot(240); break;
                        case 2: shooting = shoot(360); break;
                    }
                }


//LEDS



                telemetry.addData("indexRight", indexRight.getPower());
                telemetry.addData("indexLeft", indexLeft.getPower());





                //Turret
                //turret.setPower(0.5*gamepad2.right_stick_x);
                if(gamepad2.right_stick_button && flywheelRight.getPower() != 0 && !rightStickButt){
                    flywheelOffset2 -= 0.005;
                }
                rightStickButt = gamepad2.right_stick_button;

                //Flywheels
                if (!dpadPressed &&
                        (
                                (shotDistance == 3 && gamepad2.dpad_up) ||
                                (shotDistance == 0 && gamepad2.dpad_down) ||
                                (shotDistance == 1 && gamepad2.dpad_right) ||
                                (shotDistance == 2 && gamepad2.dpad_left))) {
                    flywheelPower = 0;
                    shotDistance = -1;
                } else if (gamepad2.dpad_down && !dpadPressed) {
                    flywheelPower = -0.4;
                    shotDistance = 0;
                } else if (gamepad2.dpad_right && !dpadPressed) {
                    flywheelPower = -0.425;
                    shotDistance = 1;
                } else if (gamepad2.dpad_left && !dpadPressed) {
                    flywheelPower = -0.5;
                    shotDistance = 2;
                } else if (gamepad2.dpad_up && !dpadPressed){
                    flywheelPower = -1;
                    shotDistance = 3;
                }
                if(gamepad2.right_trigger != 0){
                    flywheelPower = 0.25;
                } else if (gamepad2.left_trigger != 0) {
                    flywheelPower = 0;
                }

                flywheelOffset = Math.max(((time / 120000) * -0.05), -0.05);
                flywheelLeft.setPower(flywheelPower == 0 ? 0 : flywheelOffset + flywheelPower + flywheelOffset2);
                flywheelRight.setPower(flywheelPower == 0 ? 0 :flywheelOffset + flywheelPower + flywheelOffset2);
                telemetry.addData("offset", flywheelOffset);

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
                    solid = white;
                } else if(gamepad2.left_stick_y > 0){ // ball out
                    intake.setPower(1);
                    solid = red;
                } else if(gamepad2.left_stick_button){
                    intake.setPower(0);
                    solid = red;
                }
                if(prevColor != solid){
                    prevColor = solid;
                    prism.clearAllAnimations();
                    prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solid);
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
                telemetry.addData("Index 1: ", sort[0]);
                telemetry.addData("Index 2: ", sort[1]);
                telemetry.addData("Index 3: ", sort[2]);
                telemetry.addData("Spindexer Angle: ", getAngle());
                telemetry.addData("Spindexer Voltage(3.3): ", spindexerEncoder.getVoltage());
                telemetry.addData("index", index);


                telemetry.addData(" \n\n\n Clear", colorSensor.alpha());
                telemetry.addData("Red  ", colorSensor.red());
                telemetry.addData("Green", colorSensor.green());
                telemetry.addData("Blue ", colorSensor.blue());
                telemetry.addData("FrontRight", frontright.getCurrentPosition());
                telemetry.addData("FrontLeft", frontleft.getCurrentPosition());
                telemetry.addData("BackRight", backright.getCurrentPosition());
                telemetry.addData("BackLeft", backleft.getCurrentPosition());


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
    public double getAngle() {
        double voltage = spindexerEncoder.getVoltage();
        return (voltage / 3.3) * 360.0;
    }



    //Spins only right as to sort without firing a ball

    public boolean sort(double targetAngle) {
        double current = getAngle();
        double error = (((current - targetAngle) + 360) % 360);
        telemetry.addData("current", current);
        telemetry.addData("tart", targetAngle);
        telemetry.addData("comp", (((current - targetAngle) + 360)));
        telemetry.addData("error", error);

        // Stop condition: if within 5 degrees
        if (error < 10 || (current - targetAngle < 0 && targetAngle == 78) || (targetAngle == 195 && current - targetAngle < 0) || (targetAngle == 317 && current > 245 && current < 317)) {
            indexLeft.setPower(0);
            indexRight.setPower(0);
            return true; // finished
        } else {
            // Move in negative direction only
            double power = System.currentTimeMillis() - sortTime > 20 ? Math.max(0.1, error / 360) : 1;
            indexLeft.setPower(-power);
            indexRight.setPower(-power);
            return false; // still moving
        }
    }





    //Spins only left to fire ball
    public boolean shoot(double targetAngle) {
        /*double current = getAngle();
        double error = (targetAngle - current + 360) % 360;
        telemetry.addData("current", current);
        telemetry.addData("tart", targetAngle);
        telemetry.addData("comp", (((current - targetAngle) + 360)));
        telemetry.addData("error", error);
        if(error < 10 || error > 160) {
            indexLeft.setPower(0);
            indexRight.setPower(0);
            return true;
        } else {
            double power = Math.max(0.175, error / 360);
            indexLeft.setPower(power);
            indexRight.setPower(power);
            return false;
        }*/
        if(System.currentTimeMillis() - sortTime < 120){
            indexLeft.setPower(1);
            indexRight.setPower(1);
            return false;
        }else {
            indexLeft.setPower(0);
            indexRight.setPower(0);
            return true;
        }

    }

    public double angleDifference(double targetAngle) {
        return (targetAngle - getAngle()) < 0 ? targetAngle - getAngle() + 360 : targetAngle - getAngle();
    }




}