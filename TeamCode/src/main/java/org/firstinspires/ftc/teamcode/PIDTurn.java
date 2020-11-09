package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;

// TODO: Uncomment if using FTC dashboard (only works with Android Studio) (must have FTC dashboard imported in gradle)
// import com.acmerobotics.dashboard.FtcDashboard;



// FOR MECANUM DRIVETRAINS
/* SETUP TO FETCH MOTORS FOR YOU (BUT DEPENDS ON CONVENTIONAL MOTOR NAMES:)
 * leftFront
 * rightFront
 * leftRear
 * rightRear
 */
/* SETUP TO FETCH IMU FOR YOU (BUT DEPENDS ON CONVENTIONAL IMU NAME:)
 * imu
 */


public class PIDTurn {

    // get elapsed time
    private ElapsedTime PIDTimer = new ElapsedTime();

    // i
    private double integral = 0;

    // counter
    private double repetitions = 0;

    // get FIRST pid coefficients to hold pid generated coefficients
    private static PIDCoefficients testPID = new PIDCoefficients(0,0,0);

    // TODO: Uncomment if using FTC dashboard
    // FtcDashboard dashboard;

    // constructor
    public PIDTurn() {

    }

    // get motors
    private DcMotor leftFront   = null;
    private DcMotor rightFront  = null;
    private DcMotor leftRear    = null;
    private DcMotor rightRear   = null;
    private HardwareMap hwMap = null;

    // get imu
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    BNO055IMU.Parameters imuParameters;

    public void init() {
        // Define and Initialize Motors.  Assign Names that match the setup on the RC Phone
        leftFront   = hwMap.dcMotor.get("leftfront");
        rightFront  = hwMap.dcMotor.get("rightfront");
        leftRear     = hwMap.dcMotor.get("leftrear");
        rightRear    = hwMap.dcMotor.get("rightrear");
        leftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        // Define and Initalize IMU. Assign Names that match the setup on the RC Phone
        imu = hwMap.get(BNO055IMU.class, "imu 1");
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
    }

    public void runOpMode() {
        // TODO: Uncomment if using FTC dashboard
        // dashboard = FtcDashboard.getInstance();
    }

    // main pid turn with imu method
    public void turnPID(double targetAngle, double firstAngle) {
            double firstError = targetAngle - firstAngle;
            // first error used similarly to as placeholder
            double error = firstError;
            double lastError = 0;
            // error for use to stop while loop
            double imuError;
            /* ONLY FOR MOTOR TICKS IF NEEDED; USUALLY IGNORE
             * Comparison value dependent on motor tick count
             * Higher end motor tick count: higher value
             * Lower end motor tick count: lower value
             */
            while (error < targetAngle /*Modify if needed*/) {
                // DEV TODO: Look into Android Studio error for imu angular orientation LINES (125, 135 *subject to change*)
                error = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES) + firstError;
                double changeInError = lastError - error;
                integral += changeInError * PIDTimer.time();
                double derivative = changeInError / PIDTimer.time();
                double P = testPID.p * error;
                double I = testPID.i * integral;
                double D = testPID.d * derivative;
                leftFront.setPower(P + I + D);
                rightFront.setPower(-P + -I + -D);
                leftRear.setPower(P + I + D);
                rightRear.setPower(-P + -I + -D);
                error = lastError;
                PIDTimer.reset();
                imuError = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES) + firstError;
            }
    }
}
