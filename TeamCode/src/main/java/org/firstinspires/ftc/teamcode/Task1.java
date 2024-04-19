package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Task 1")
public class Task1 extends LinearOpMode {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;

    private void drive() {
        double leftSpeed = gamepad1.left_stick_y + gamepad1.left_stick_x;
        double rightSpeed = gamepad1.left_stick_y - gamepad1.left_stick_x;
        leftMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);
    }

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        if(opModeIsActive()) {
            while(opModeIsActive()) {
                drive();
            }
        }
    }
}
