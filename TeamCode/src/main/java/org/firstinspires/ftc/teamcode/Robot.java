package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot {
    public DcMotor FrontLeftDrive = null;
    public DcMotor FrontRightDrive = null;
    public DcMotor RearLeftDrive = null;
    public DcMotor RearRightDrive = null;
    public CRServo TestServo = null;
    public CRServo TestServo2 = null;
    public Servo IntakeServo = null;
    public Servo ClawServoR = null;
    public Servo ClawServoL = null;
    public DcMotor IntakeMotor = null;
    public BNO055IMU imu;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public void init(HardwareMap ahwMap){
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        FrontLeftDrive = hwMap.get(DcMotor.class, "FL_DCmotor");
        FrontRightDrive = hwMap.get(DcMotor.class, "FR_DCmotor");
        RearLeftDrive = hwMap.get(DcMotor.class, "RL_DCmotor");
        RearRightDrive = hwMap.get(DcMotor.class, "RR_DCmotor");
  //      TestServo = hwMap.get(CRServo.class,"TestServo");
  //      TestServo2 = hwMap.get(CRServo.class,"TestServo2");
        ClawServoR = hwMap.get(Servo.class, "ClawServoR");
        ClawServoL = hwMap.get(Servo.class, "ClawServoL");
        IntakeMotor = hwMap.get(DcMotor.class, "IntakeMotor");
        IntakeServo = hwMap.get(Servo.class,"IntakeServo");

        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        RearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        RearRightDrive.setDirection(DcMotor.Direction.FORWARD);
        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);

        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        RearLeftDrive.setPower(0);
        RearRightDrive.setPower(0);
        IntakeMotor.setPower(0);

        FrontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hwMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);
    }

    public void drive(double power, int EncoderCounts){
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftDrive.setTargetPosition(EncoderCounts);
        FrontRightDrive.setTargetPosition(EncoderCounts);
        RearLeftDrive.setTargetPosition(EncoderCounts);
        RearRightDrive.setTargetPosition(EncoderCounts);
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeftDrive.setPower(power);
        FrontRightDrive.setPower(power);
        RearLeftDrive.setPower(power);
        RearRightDrive.setPower(power);
        while (FrontLeftDrive.isBusy() || FrontRightDrive.isBusy() || RearLeftDrive.isBusy() || RearRightDrive.isBusy()) {
            //telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
            //        FrontLeftDrive.getCurrentPosition(),
            //            FrontRightDrive.getCurrentPosition(),
            //          RearLeftDrive.getCurrentPosition(),
            //        RearRightDrive.getCurrentPosition());
            //telemetry.update();
        }
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        RearLeftDrive.setPower(0);
        RearRightDrive.setPower(0);

        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void strafe(double power, int EncoderCounts){
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeftDrive.setTargetPosition(-1 * EncoderCounts);
        FrontRightDrive.setTargetPosition(EncoderCounts);
        RearLeftDrive.setTargetPosition(EncoderCounts);
        RearRightDrive.setTargetPosition(-1 * EncoderCounts);
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeftDrive.setPower(-1 * power);
        FrontRightDrive.setPower(power);
        RearLeftDrive.setPower(power);
        RearRightDrive.setPower(-1 * power);
        while (FrontLeftDrive.isBusy() || FrontRightDrive.isBusy() || RearLeftDrive.isBusy() || RearRightDrive.isBusy()) {

        }
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        RearLeftDrive.setPower(0);
        RearRightDrive.setPower(0);

        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void gyroTurn(double speed, double angle,double coeff) {
        double error;
        double steer;
        double leftSpeed, rightSpeed;
        boolean onTarget = false;
        error = getError(angle);
        while (Math.abs(error) > 1) {
            steer = Range.clip(coeff * error, -speed, speed);
            rightSpeed = steer;
            leftSpeed = -rightSpeed;
            FrontLeftDrive.setPower(leftSpeed);
            RearLeftDrive.setPower(leftSpeed);
            FrontRightDrive.setPower(rightSpeed);
            RearRightDrive.setPower(rightSpeed);
            error = getError(angle);
        }
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        RearLeftDrive.setPower(0);
        RearRightDrive.setPower(0);
    }

    public double getError (double targetAngle){
        double angleError;
        Orientation orientation = imu.getAngularOrientation(
                AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        angleError = targetAngle - orientation.thirdAngle;

        if(angleError > 180){
            angleError-=360;
        }
        if (angleError <= -180){
            angleError +=360;
        }
        return angleError;
    }
}
