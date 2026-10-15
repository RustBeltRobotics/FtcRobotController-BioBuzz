package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp(name = "Basic Motor Test", group = "Robot")
public class BasicMotorTest extends LinearOpMode {
    DcMotorEx feeder;
    DcMotorEx intake;

    VoltageSensor voltageSensor;
    WebTelemetryStreamer webTelemetryStreamer;
    WebInterface webInterface;

    @Override
    public void runOpMode() {
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        webTelemetryStreamer = new WebTelemetryStreamer(8886);
        webTelemetryStreamer.setChannels(new ArrayList<String>(Arrays.asList("main_current", "main_speed", "voltage", "backspin_current", "backspin_speed", "rpm_i", "ratio_i", "rpm_back_i")));
        Thread webTelemetryStreamerThread = new Thread(webTelemetryStreamer);
        webTelemetryStreamerThread.start();

        webInterface = new WebInterface(8885);
        webInterface.addParameter("main_rpm", 0.0, 3000);
        webInterface.addParameter("backspin_ratio", 1.0, 5.0);

        PIDFCoefficients defaults_shooter = ((DcMotorEx) feeder).getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        defaults_shooter.p = 118; // set default
        defaults_shooter.i = 1.45;
        defaults_shooter.d = 4.9;
        webInterface.addParameter("Kp_main", defaults_shooter.p, 150);
        webInterface.addParameter("Ki_main", defaults_shooter.i, 20);
        webInterface.addParameter("Kd_main", defaults_shooter.d, 20);
        webInterface.addParameter("Kf_main", defaults_shooter.f, 20);

        PIDFCoefficients defaults_shooter2 = ((DcMotorEx) intake).getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        defaults_shooter2.p = 100; // set default
        defaults_shooter2.i = 3.8;
        defaults_shooter2.d = 0;
        webInterface.addParameter("Kp_back", defaults_shooter2.p, 150);
        webInterface.addParameter("Ki_back", defaults_shooter2.i, 20);
        webInterface.addParameter("Kd_back", defaults_shooter2.d, 20);
        webInterface.addParameter("Kf_back", defaults_shooter2.f, 20);


        Thread webInterfaceThread = new Thread(webInterface);
        webInterfaceThread.start();

        feeder.setDirection(DcMotor.Direction.REVERSE);
        feeder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        telemetry.addData("laaa", feeder.getMotorType().getTicksPerRev());
//        telemetry.update();
//
        waitForStart();

        int hardcoded_ticks_per_revolution = 28;

        int loopcounter = 0;
        while (opModeIsActive()) {
            double rpm = webInterface.getParameter("main_rpm");
            double ratio = webInterface.getParameter("backspin_ratio");
            feeder.setVelocity(rpm * hardcoded_ticks_per_revolution / 60.0);
            intake.setVelocity(rpm * ratio * hardcoded_ticks_per_revolution / 60.0);

            ((DcMotorEx) feeder).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                    webInterface.getParameter("Kp_main"),
                    webInterface.getParameter("Ki_main"),
                    webInterface.getParameter("Kd_main"),
                    webInterface.getParameter("Kf_main")
            ));

            ((DcMotorEx) intake).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                    webInterface.getParameter("Kp_back"),
                    webInterface.getParameter("Ki_back"),
                    webInterface.getParameter("Kd_back"),
                    webInterface.getParameter("Kf_back")
            ));

//            feeder.setPower(5.0);
//            intake.setPower(5.0);


//            if (loopcounter++ % 2 == 0) {

            double horrible_coefficient = 20;

                webTelemetryStreamer.sendData("main_current", feeder.getCurrent(CurrentUnit.MILLIAMPS) / 1000);
                webTelemetryStreamer.sendData("main_speed", horrible_coefficient * feeder.getVelocity(AngleUnit.DEGREES) / 6.0);
                webTelemetryStreamer.sendData("voltage", voltageSensor.getVoltage());

                webTelemetryStreamer.sendData("backspin_current", intake.getCurrent(CurrentUnit.MILLIAMPS) / 1000);
                webTelemetryStreamer.sendData("backspin_speed", horrible_coefficient * intake.getVelocity(AngleUnit.DEGREES) / 6.0);

                webTelemetryStreamer.sendData("rpm_i", rpm);
                webTelemetryStreamer.sendData("ratio_i", ratio);
                webTelemetryStreamer.sendData("rpm_back_i", rpm * ratio);

            webTelemetryStreamer.sendUpdateFrame();
//            }

            telemetry.addData("main_rpm setpoint", rpm);
            telemetry.addData("backspin_rpm setpoint", rpm * ratio);
            telemetry.update();
        }

        webInterface.stop();

        try {
            webTelemetryStreamer.stop();
        } catch (Exception ignored) {}
    }
}