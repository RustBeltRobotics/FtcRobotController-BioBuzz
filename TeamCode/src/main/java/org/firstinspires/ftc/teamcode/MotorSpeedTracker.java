package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MotorSpeedTracker {
    DcMotor motor;
    DcMotorEx motorex;

    int lastEncoderCount;

    ElapsedTime timer;

    double cpsraw;
    double rpm;
    int countsPerRev;

    int countdiff;

    MotorSpeedTracker(DcMotor motor, int countsPerRev) { // countsPerRev = 28 for ungeared rev hex motor
        this.motor = motor;
        this.motorex = (DcMotorEx) motor;

        this.lastEncoderCount = this.motorex.getCurrentPosition();
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();

        this.countsPerRev = countsPerRev;
        this.countdiff = 0;
    }

    double update() {
        int currentEncoderCount = this.motorex.getCurrentPosition();
        double elapsed = timer.milliseconds();
        timer.reset();

        this.countdiff = currentEncoderCount - this.lastEncoderCount;
        this.cpsraw = 1000.0 * (this.countdiff/elapsed);
        this.rpm = (this.cpsraw/this.countsPerRev) * 60;

        this.lastEncoderCount = currentEncoderCount;
        return this.rpm;
    }

    double rpm() {
        return this.rpm;
    }
}
