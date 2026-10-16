/* Copyright (c) 2025 FIRST. All rights reserved.
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

import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.IMU;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.matrices.VectorF;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import java.io.IOException;
import java.util.Locale;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@TeleOp(name = "Robot: Field Relative Mecanum Drive MAIN YAY", group = "Robot")
public class SVPUtiliserCeModeTeleop extends LinearOpMode {
    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    DcMotor shooterL;


    DcMotor intake;

    DcMotor feeder;

    DcMotor shooterR;

//    Rev9AxisImu imu;

    IMU imu;

    Orientation angles;


    WebTelemetryStreamer webTelemetryStreamer;

    WebInterface webInterface;

    boolean shooterToggle = false;
    boolean lastGamepadX = false;

    Toggler aToggler = new Toggler();
    Toggler bToggler = new Toggler();


    DriveController driveController;

    double yawZero;

    SparkFunOTOS myOtos;

    // This declares the IMU needed to get the current direction the robot is facing
//    IMU imu;

    PIDController otosXController;
    PIDController otosYController;

    PIDController otosRotationController;
    PIDController apriltagRotationController;

    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    boolean doingOtosTest = false;

    @Override
    public void runOpMode() {
//        Rev9AxisImu.Parameters parameters = new Rev9AxisImu.Parameters(new Rev9AxisImuOrientationOnRobot(Rev9AxisImuOrientationOnRobot.LogoFacingDirection.UP, Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.BACKWARD));
//        imu = hardwareMap.get(Rev9AxisImu.class, "external_imu");

        Rev9AxisImu.Parameters parameters = new Rev9AxisImu.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu = hardwareMap.get(IMU.class, "imu"); // internal IMU
        imu.initialize(parameters);

//        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
//        myOtos.resetTracking();


        System.out.println("MAIN ACTUAL OPCODE YAY A");

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        shooterL = hardwareMap.get(DcMotor.class, "shooterL");
        shooterR = hardwareMap.get(DcMotor.class, "shooterR");
        intake = hardwareMap.get(DcMotor.class, "intake");
        feeder = hardwareMap.get(DcMotor.class, "feeder");

        webTelemetryStreamer = new WebTelemetryStreamer(8886);
        webTelemetryStreamer.setChannels(new ArrayList<String>(Arrays.asList("shooterR_speed", "shooterR_current", "shooterL_speed", "shooterL_current", "intake_speed", "intake_current", "feeder_speed", "feeder_current", "SPshooter_speed", "SPfeeder_speed", "SPintake_speed")));
        Thread webTelemetryStreamerThread = new Thread(webTelemetryStreamer);
        webTelemetryStreamerThread.start(); // start server

        webInterface = new WebInterface(8885);
        // webInterface.addParameter("shooter_power", 0.6);
        // webInterface.addParameter("feeder_power", 0.6); //0.4

        webInterface.addParameter("shooter_speed", 2500, 5000);
        webInterface.addParameter("feeder_speed", 2500, 4000);
        webInterface.addParameter("intake_speed", 2500, 5000);

        webInterface.addParameter("Kp_drive", 0.01);
        webInterface.addParameter("Ki_drive", 0.00);
        webInterface.addParameter("Kd_drive", 0.00);



        PIDFCoefficients defaults_shooter =((DcMotorEx) shooterL).getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        defaults_shooter.p = 90; // set default

        webInterface.addParameter("Kp_shooter", defaults_shooter.p);
        webInterface.addParameter("Ki_shooter", defaults_shooter.i);
        webInterface.addParameter("Kd_shooter", defaults_shooter.d);
        webInterface.addParameter("Kf_shooter", defaults_shooter.f);


        // do not adjust these
        otosXController = new PIDController(0.0, 0.0, 0.0);
        otosYController = new PIDController(0.0, 0.0, 0.0);
        otosRotationController = new PIDController(0.0, 0.0, 0.0);


        apriltagRotationController = new PIDController(0.0, 0.0, 0.0);

        // ---

        // adjust these
        webInterface.addParameter("Kp_otos", 0.08);
        webInterface.addParameter("Ki_otos", 0.00003);
        webInterface.addParameter("Kd_otos", 0.35);

        webInterface.addParameter("Kp_otosh", 0.004);
        webInterface.addParameter("Ki_otosh", 0.00001);
        webInterface.addParameter("Kd_otosh", 0.0001);


        webInterface.addParameter("Kp_aprilh", 0.0007);
        webInterface.addParameter("Ki_aprilh", 0.000002);
        webInterface.addParameter("Kd_aprilh", 0.004);


//        webInterface.addParameter("dbsizec", 0.3);
//        webInterface.addParameter("dbdepthc", 0.3);

        Thread webInterfaceThread = new Thread(webInterface);
        webInterfaceThread.start(); // start server

//        SoundPlayer soundPlayer = new SoundPlayer(1, 4096);
//        soundPlayer.play()
//        SoundPlayer.getInstance().startPlaying();



        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feeder.setDirection(DcMotor.Direction.REVERSE);
        feeder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterR.setDirection(DcMotor.Direction.FORWARD);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setDirection(DcMotor.Direction.REVERSE);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



//        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

//        RevHubOrientationOnRobot orientationOnRobot = new
//                RevHubOrientationOnRobot(logoDirection, usbDirection);
//        imu.initialize(new IMU.Parameters(orientationOnRobot));


        waitForStart();

        driveController = new DriveController(imu, frontRightDrive, frontLeftDrive, backLeftDrive, backRightDrive, new PIDController(0.02, 0.001, 0.001));//new PIDController(0.0, 0.0, 0.0));
        driveController.init();

        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveController.yawZero = 0.0 - angles.firstAngle;
//        yawZero = 0.0 - angles.firstAngle;

//        initAprilTag();

        int loopcounter = 0;
        while (opModeIsActive()) {
            loop2(loopcounter++);
        }

//        try {
            webInterface.stop();
//        } catch (IOException e) {}

        try {
            webTelemetryStreamer.stop();
        } catch (IOException e) {}
    }

    private double avg(double a, double b) {
        return (a + b)/2;
    }

    private double boolToDoubleBecauseItWontCast(boolean input) {
        return input ? 1.0 : 0.0;
    }

    public void loop2(int loopcounter) {
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");

        boolean doingOtosTest = gamepad1.back;


//        doingOtosTest = doingOtosTest ^ gamepad1.backWasPressed();

//        if (gamepad1.leftBumperWasPressed()) {
//            doingOtosTest = true;
//        }
//        if (gamepad1.leftBumperWasReleased()) {
//            doingOtosTest = false;
//        }

        driveController.drivingPID.setCoefs(
                webInterface.getParameter("Kp_drive"),
                webInterface.getParameter("Ki_drive"),
                webInterface.getParameter("Kd_drive")
        );

//        driveController.drivingPID.deadbandSizeCoef = webInterface.getParameter("dbsizec");
//        driveController.drivingPID.deadbandDepthCoef = webInterface.getParameter("dbdepthc");

//        driveController.drivingPID.setDeadbandStuff(webInterface.getParameter("dbsizec"), webInterface.getParameter("dbdepthc"));

        ((DcMotorEx) shooterL).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(
                        webInterface.getParameter("Kp_shooter"),
                        webInterface.getParameter("Ki_shooter"),
                        webInterface.getParameter("Kd_shooter"),
                        webInterface.getParameter("Kf_shooter")
                )
        );

        ((DcMotorEx) shooterR).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(
                        webInterface.getParameter("Kp_shooter"),
                        webInterface.getParameter("Ki_shooter"),
                        webInterface.getParameter("Kd_shooter"),
                        webInterface.getParameter("Kf_shooter")
                )
        );

//        System.out.print("hashfuaehwoigfheraoiwghioe: ");
//        System.out.println(webInterface.getParameter("Kp_drive"));
//        System.out.println(driveController.drivingPID.Kp);
//        System.out.println(driveController.drivingPID.Ki);
//        System.out.println(driveController.drivingPID.Kd);


        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("connection", imu.getConnectionInfo());

        // wrong labels
        telemetry.addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.addData("roll", formatAngle(angles.angleUnit, angles.secondAngle));
        telemetry.addData("pitch", formatAngle(angles.angleUnit, angles.thirdAngle));

//        SparkFunOTOS.Pose2D otosPos = myOtos.getPosition();


//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            }
//        }


//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
//        telemetry.addData("FPS: ", visionPortal.getFps());

//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null && detection.ftcPose != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }

        // send slower to stop network getting backed up (maybe it would help if I don't flush data inside of sendData - oh wait I'm already doing that)
        if (true) { //(loopcounter % 3 == 0) {
//            if (currentDetections.size() > 0) {
//                AprilTagDetection detection = currentDetections.get(0);
//                webTelemetryStreamer.sendData("xa", detection.ftcPose.x);
//                webTelemetryStreamer.sendData("ya", detection.ftcPose.y);
//                webTelemetryStreamer.sendData("za", detection.ftcPose.z);
//            }

            double horrible_coefficient = 20;

            webTelemetryStreamer.sendData("shooterL_current", ((DcMotorEx) shooterL).getCurrent(CurrentUnit.MILLIAMPS) / 1000);
            webTelemetryStreamer.sendData("shooterL_speed", horrible_coefficient * ((DcMotorEx) shooterL).getVelocity(AngleUnit.DEGREES) / 6.0);

            webTelemetryStreamer.sendData("shooterR_current", ((DcMotorEx) shooterR).getCurrent(CurrentUnit.MILLIAMPS) / 1000);
            webTelemetryStreamer.sendData("shooterR_speed", horrible_coefficient * ((DcMotorEx) shooterR).getVelocity(AngleUnit.DEGREES) / 6.0);

            webTelemetryStreamer.sendData("feeder_current", ((DcMotorEx) feeder).getCurrent(CurrentUnit.MILLIAMPS) / 1000);
            webTelemetryStreamer.sendData("feeder_speed", horrible_coefficient * ((DcMotorEx) feeder).getVelocity(AngleUnit.DEGREES) / 6.0);

            webTelemetryStreamer.sendData("intake_current", ((DcMotorEx) intake).getCurrent(CurrentUnit.MILLIAMPS) / 1000);
            webTelemetryStreamer.sendData("intake_speed", horrible_coefficient * ((DcMotorEx) intake).getVelocity(AngleUnit.DEGREES) / 6.0);

            webTelemetryStreamer.sendData("SPintake_speed", webInterface.getParameter("intake_speed"));
            webTelemetryStreamer.sendData("SPshooter_speed", webInterface.getParameter("shooter_speed"));
            webTelemetryStreamer.sendData("SPfeeder_Speed", webInterface.getParameter("feeder_speed"));



            //            webTelemetryStreamer.sendData("voltage", voltageSensor.getVoltage());

//            webTelemetryStreamer.sendData("backspin_current", shooterR.getCurrent(CurrentUnit.MILLIAMPS) / 1000);
//            webTelemetryStreamer.sendData("backspin_speed", horrible_coefficient * intake.getVelocity(AngleUnit.DEGREES) / 6.0);

//            webTelemetryStreamer.sendData("rpm_i", rpm);
//            webTelemetryStreamer.sendData("ratio_i", ratio);
//            webTelemetryStreamer.sendData("rpm_back_i", rpm * ratio);

            webTelemetryStreamer.sendUpdateFrame();


//            webTelemetryStreamer.sendData("heading", angles.firstAngle);
//            webTelemetryStreamer.sendData("roll", angles.secondAngle);
//            webTelemetryStreamer.sendData("pitch", angles.thirdAngle);
//
//            webTelemetryStreamer.sendData("PIDTarget", (180/3.14159) * driveController.drivingPID.target);
//            webTelemetryStreamer.sendData("PIDOutput", driveController.drivingPID.output);
//
//            webTelemetryStreamer.sendData("x", gamepad1.left_stick_x * 100.0);
//            webTelemetryStreamer.sendData("y", gamepad1.left_stick_y * 100.0);
//            webTelemetryStreamer.sendData("theta", gamepad1.right_stick_x * 100.0);
//
//            webTelemetryStreamer.sendData("current_frontLeftDrive", ((DcMotorEx) frontLeftDrive).getCurrent(CurrentUnit.MILLIAMPS));
//            webTelemetryStreamer.sendData("current_frontRightDrive", ((DcMotorEx) frontRightDrive).getCurrent(CurrentUnit.MILLIAMPS));
//            webTelemetryStreamer.sendData("current_backLeftDrive", ((DcMotorEx) backLeftDrive).getCurrent(CurrentUnit.MILLIAMPS));
//            webTelemetryStreamer.sendData("current_backRightDrive", ((DcMotorEx) backRightDrive).getCurrent(CurrentUnit.MILLIAMPS));

//            webTelemetryStreamer.sendData("speed_shooter", -((DcMotorEx) shooterL).getVelocity(AngleUnit.DEGREES));
//
//            webTelemetryStreamer.sendData("speed_intake", -((DcMotorEx) intake).getVelocity(AngleUnit.DEGREES));
//            webTelemetryStreamer.sendData("speed_feeder", -((DcMotorEx) feeder).getVelocity(AngleUnit.DEGREES));
//            webTelemetryStreamer.sendData("speed_feeder2", ((DcMotorEx) shooterR).getVelocity(AngleUnit.DEGREES));
//
//
//            webTelemetryStreamer.sendData("current_shooter", ((DcMotorEx) shooterL).getCurrent(CurrentUnit.MILLIAMPS));
//            webTelemetryStreamer.sendData("current_intake", ((DcMotorEx) intake).getCurrent(CurrentUnit.MILLIAMPS));
//            webTelemetryStreamer.sendData("current_feeder", ((DcMotorEx) feeder).getCurrent(CurrentUnit.MILLIAMPS));
//            webTelemetryStreamer.sendData("current_feeder2", ((DcMotorEx) shooterR).getCurrent(CurrentUnit.MILLIAMPS));


//            webTelemetryStreamer.sendData("sf_x", otosPos.x);
//            webTelemetryStreamer.sendData("sf_y", otosPos.y);
//            webTelemetryStreamer.sendData("sf_h", otosPos.h);

            // get voltage sensor (there are two one is on the expansion hub and this is not ideal as it does not specify which one to use)
//            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
////                webTelemetryStreamer.sendData("battery_voltage", sensor.getVoltage() * 100); // multiply by 100 so it is in a similar range to other values
//                break;
//            }
        }

        // TODO: make opmode just to test every motors' current draw individually under no load

        double invert_all_emergency = ((Math.min((gamepad1.left_bumper ? 1 : 0) + (gamepad2.left_bumper ? 1 : 0), 1.0)) * 2 - 1);

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
//        if (gamepad1.a) {
//            imu.resetYaw();
//        }
        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)

//        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // 90° rotation
//        drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (!doingOtosTest) {
            if (!gamepad1.right_bumper && false) {
                // working:
                driveController.driveFieldRelative(jsrc(gamepad1.left_stick_y), jsrc(gamepad1.left_stick_x), jsrc(gamepad1.right_stick_x));
                // should attempt to correct for unintentional rotation
                //            driveController.driveFieldRelativeAuto(jsrc(gamepad1.left_stick_y), jsrc(gamepad1.left_stick_x), jsrc(gamepad1.right_stick_x));
            } else {
//                telemetry.addLine("RIGHT BUMPER");
//                drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

                // doingApriltagRotationTest
                telemetry.addLine("apriltag");

//                AprilTagDetection detection = currentDetections.stream()
//                        .filter(d -> d.id == 32) // id 32
//                        .findFirst()
//                        .orElse(null);

//                if (detection != null) {
//                    apriltagRotationController.setTarget(300); // 300 pixels
//
//                    double rotationSignal = apriltagRotationController.loop(detection.center.x);

//                    driveController.driveFieldRelative(jsrc(gamepad1.left_stick_y), jsrc(gamepad1.left_stick_x), rotationSignal + jsrc(gamepad1.right_stick_x));
//                } else {
                    driveController.driveFieldRelative(jsrc(gamepad1.left_stick_y), jsrc(gamepad1.left_stick_x), jsrc(gamepad1.right_stick_x));
//                }
            }
        }




//        if (gamepad1.left_bumper) {
//            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//        } else {
//            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//        }

        if (gamepad1.x && !lastGamepadX) {
            shooterToggle = !shooterToggle;
        }

        if (gamepad1.dpad_up) {
            driveController.yawZero = 0.0 - angles.firstAngle;
        }

        lastGamepadX = gamepad1.x;


        aToggler.update(gamepad1.a);
//        bToggler.update(gamepad1.left_bumper);



        // double shooterPowerCoef = webInterface.getParameter("shooter_power"); // 0.80;
        // double upperWheelPower = webInterface.getParameter("feeder_power");  // 0.3
        double restPowerLevel = 1.9; // 1.9


        double shooter_speed = webInterface.getParameter("shooter_speed");


        int hardcoded_ticks_per_revolution = 28;

        double reversalSignal = 1 - (2 * boolToDoubleBecauseItWontCast(gamepad1.b));

        ((DcMotorEx) intake).setVelocity(reversalSignal * boolToDoubleBecauseItWontCast(gamepad1.left_bumper) * webInterface.getParameter("intake_speed"));
        ((DcMotorEx) feeder).setVelocity(reversalSignal * boolToDoubleBecauseItWontCast(gamepad1.right_bumper) * webInterface.getParameter("feeder_speed"));

        double shooterSpeed = boolToDoubleBecauseItWontCast(aToggler.currentState) * shooter_speed * hardcoded_ticks_per_revolution / 60.0;
        ((DcMotorEx) shooterR).setVelocity(shooterSpeed);
        ((DcMotorEx) shooterL).setVelocity(shooterSpeed);

        
        otosXController.setCoefs(
                webInterface.getParameter("Kp_otos"),
                webInterface.getParameter("Ki_otos"),
                webInterface.getParameter("Kd_otos")
        );

        otosYController.setCoefs(
                webInterface.getParameter("Kp_otos"),
                webInterface.getParameter("Ki_otos"),
                webInterface.getParameter("Kd_otos")
        );

        otosRotationController.setCoefs(
                webInterface.getParameter("Kp_otosh"),
                webInterface.getParameter("Ki_otosh"),
                webInterface.getParameter("Kd_otosh")
        );

        apriltagRotationController.setCoefs(
                webInterface.getParameter("Kp_aprilh"),
                webInterface.getParameter("Ki_aprilh"),
                webInterface.getParameter("Kd_aprilh")
        );

        // OTOS position PID test:
//        if (doingOtosTest) {
//            telemetry.addLine("otosTest");
//
//            otosXController.setTarget(2);
//            otosYController.setTarget(3);
//            otosRotationController.setTarget(0);
//
//            double xSignal = otosXController.loop(otosPos.x);
//            double ySignal = otosYController.loop(otosPos.y);
//            double rotationSignal = -otosRotationController.loop(otosPos.h);
//
//            webTelemetryStreamer.sendData("xSignal", xSignal);
//            webTelemetryStreamer.sendData("ySignal", ySignal);
//            webTelemetryStreamer.sendData("rotationSignal", rotationSignal);
//
//            driveController.driveFieldRelative(xSignal, ySignal, rotationSignal);
//        }


        // telemetry.addData("shooter_power", webInterface.getParameter("shooter_power"));

//        gamepad2.resetEdgeDetection();
        gamepad1.resetEdgeDetection(); // used to toggle otos test thing
        telemetry.update();
    }

    private void initAprilTag() {
        AprilTagLibrary.Builder myLibraryBuilder = new AprilTagLibrary.Builder();
        myLibraryBuilder.addTag(21, "21", 6.5, DistanceUnit.INCH);
        myLibraryBuilder.addTag(24, "24", 6.5, DistanceUnit.INCH);
        myLibraryBuilder.addTag(11, "11", 6.5, DistanceUnit.INCH);
        myLibraryBuilder.addTag(19, "21", 6.5, DistanceUnit.INCH);
        myLibraryBuilder.addTag(20, "24", 6.5, DistanceUnit.INCH);

        AprilTagLibrary myCustomLibrary = myLibraryBuilder.build();
        aprilTag = new AprilTagProcessor.Builder().setTagLibrary(myCustomLibrary).build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    private double jsrc(double power) {
//        double a = 3.7;
//        double b = 0.43;
        double a = 3.33;
        double b = 0.35;

        return (Math.signum(power)*Math.pow(Math.abs(power), a) + (power * b)) / (1 + b);
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        // reverse angle because the robot magically got mixed up and the angle was inverted

        double yaw_corrected = ((((0.0 - angles.firstAngle) - this.yawZero) + 180.0)) % 360.0 - 180.0;

        theta = AngleUnit.normalizeRadians(theta + (yaw_corrected) * (3.141592653589/180));//AngleUnit.normalizeRadians(theta -
                //imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
