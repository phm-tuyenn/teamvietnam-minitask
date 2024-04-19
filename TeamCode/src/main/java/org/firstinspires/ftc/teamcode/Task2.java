package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Task 2")
public class Task2 extends LinearOpMode {
    private static final double TRACK_WIDTH = 4000; //mm
    private static final double WHEEL_C = 600; //mm
    private static final double TICKS_PER_ROT = 28; //ticks
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private DcMotorEx horizontalMotor;
    private IMU imu;

    private void drive() {
        double leftSpeed = gamepad1.left_stick_y + gamepad1.left_stick_x;
        double rightSpeed = gamepad1.left_stick_y - gamepad1.left_stick_x;
        leftMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);
    }

    private double x = 0, y = 0, heading = 0, theta = 0;
    private double prevL = 0, prevR = 0, prevH = 0;
    private void odometry() {
        double L = leftMotor.getCurrentPosition();
        double R = rightMotor.getCurrentPosition();
        double H = horizontalMotor.getCurrentPosition();

        double deltaL = ((L - prevL) / TICKS_PER_ROT) * 2.0 * Math.PI * WHEEL_C;
        double deltaR = ((R - prevL) / TICKS_PER_ROT) * 2.0 * Math.PI * WHEEL_C;
        double deltaH = ((H - prevL) / TICKS_PER_ROT) * 2.0 * Math.PI * WHEEL_C;

        x += (((deltaL + deltaR) / 2.0)) * Math.cos(theta);
        y  += (((deltaL + deltaR) / 2.0)) * Math.sin(theta);
        theta  += (deltaL - deltaR) / TRACK_WIDTH;

        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        prevL = L;
        prevR = R;
        prevH = H;
    }

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");
        horizontalMotor = hardwareMap.get(DcMotorEx.class, "horizontalMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();
        if(opModeIsActive()) {
            while(opModeIsActive()) {
                drive();
                odometry();
                telemetry.addData("x", x);
                telemetry.addData("y", y);
                telemetry.addData("theta", theta);
                telemetry.addData("heading", heading);
                telemetry.update();
            }
        }
    }
}
