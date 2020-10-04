package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CustomPID extends LinearOpMode {
    DcMotor testMotor;

    double integral = 0;
    double repetitions = 0;

    public static PIDCoefficients testPID = new PIDCoefficients(0,0,0);

    FtcDashboard dashboard;

    public static double TARGET_POS = 100; // 100 is default value

    ElapsedTime PIDTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        testMotor = hardwareMap.dcMotor.get("testMotor");

        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        moveTestMotor(TARGET_POS);

    }
    void moveTestMotor(double targetPosition) {
        double error = testMotor.getCurrentPosition();
        double lastError = 0;

        /*
         * Comparison value dependent on motor tick count
         * Higher end motor tick count: higher value
         * Lower end motor tick count: lower value
         */
        while (Math.abs(error) <= 9 /*Modify with above comments*/ && repetitions < 40 /*Modify*/) {
            error = testMotor.getCurrentPosition() - targetPosition;
            double changeInError = lastError - error;
            integral += changeInError * PIDTimer.time();
            double derivative = changeInError / PIDTimer.time();
            double P = testPID.p * error;
            double I = testPID.i * integral;
            double D = testPID.d * derivative;
            testMotor.setPower(P + I + D);
            error = lastError;
            PIDTimer.reset();
            repetitions ++;
        }
    }
}
