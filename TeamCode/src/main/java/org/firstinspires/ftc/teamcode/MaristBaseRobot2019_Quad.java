package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by michaudc on 10/8/2017.
 * Based on HardwarePushbot Code from the FTCRobotController resources
 *
 * This class models the physical structure of the robot with instances
 * of motors, servos, and sensors.
 *
 * The following are name assignments to be configured
 * on the RC Phone in the App.
 *
 * Motor channel: leftFront:        "leftfront"
 * Motor channel: rightFront:       "rightfront"
 * Motor channel: leftRear:          "leftrear"
 * Motor channel: rightRear:         "rightrear"
 * Motor channel: wobbleArm:        "wobblearm"
 * Servo Channel: leftHand:         "lefthand"
 * Servo Channel: rightHand:        "righthand"
 *
 */

public class MaristBaseRobot2019_Quad {
    /* Public Motors and Servos */
    public DcMotor leftFront   = null;
    public DcMotor rightFront  = null;
    public DcMotor leftRear    = null;
    public DcMotor rightRear   = null;
    public DcMotor wobbleArm = null;
    public Servo rightHand    = null;
    public Servo leftHand   = null;

    // Constants for Arm and Servo Operation
    public static final double MID_SERVO        =  0.5;
    public static final double ARM_UP_POWER     =  0.45;
    public static final double ARM_DOWN_POWER   = -0.45;

    // For Encoder Functions
    private double     COUNTS_PER_MOTOR_REV          = 1440 ;    // eg: TETRIX Motor Encoder
    private final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private double     WHEEL_DIAMETER_INCHES         = 4.0 ;     // For figuring circumference
    private double     COUNTS_PER_INCH               = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private double COUNTS_PER_DEGREE                 = COUNTS_PER_MOTOR_REV / 360;
    private double     DRIVE_SPEED                   = 0.6;
    private double     TURN_SPEED                    = 0.5;

    // Local OpMode members
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    // Constructor - leave this blank for now
    public MaristBaseRobot2019_Quad () {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors.  Assign Names that match the setup on the RC Phone
        leftFront   = hwMap.dcMotor.get("leftfront");
        rightFront  = hwMap.dcMotor.get("rightfront");
        leftRear     = hwMap.dcMotor.get("leftrear");
        rightRear    = hwMap.dcMotor.get("rightrear");
        wobbleArm = hwMap.dcMotor.get("wobblearm");
        leftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors


        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        leftHand = hwMap.servo.get("lefthand");
        rightHand = hwMap.servo.get("righthand");
        leftHand.setPosition(MID_SERVO);
        rightHand.setPosition(MID_SERVO);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    // Additional Functions to control Servos and motors

    public void driveStraightInches(double speed,
                                    double inches,
                                    double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Reverse inches
        inches = inches * -1;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftRearTarget = leftRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightRearTarget = rightRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            //
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftRear.setTargetPosition(newLeftRearTarget);
            rightRear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftRear.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((period.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy() )) {
                // Wait for Sequence to complete
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void pointTurnDegrees(double speed,
                                 double deg,
                                 double timeoutS) {

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Reverse inches
        deg = deg * -1;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
            newRightFrontTarget = rightFront.getCurrentPosition() - (int)(deg * COUNTS_PER_DEGREE);
            newLeftRearTarget = leftRear.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
            newRightRearTarget = rightRear.getCurrentPosition() - (int)(deg * COUNTS_PER_DEGREE);

            //
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftRear.setTargetPosition(newLeftRearTarget);
            rightRear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftRear.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((period.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy() )) {
                // Wait for Sequence to complete
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

        /*
        int newLeftTarget;
        int newRightTarget;
        // Reverse inches
        deg = deg * -1;
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > TURN_SPEED) {
            speed = TURN_SPEED; //
        }
        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot
            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
            newRightTarget = rightMotor.getCurrentPosition() - (int)(deg * COUNTS_PER_DEGREE);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            period.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((period.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {
                // Wait for Sequence to complete
            }
            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
        */
    }



}