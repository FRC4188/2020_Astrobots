package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Funxd", group="Linear Opmode")
public class MechDriveAstrobots extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frMotor, flMotor, brMotor, blMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        waitForStart();

        while (opModeIsActive()) {
            drivetrain(0.5,0, 0);
        }
    }

    private void controllerTest() {
        double forwardController = -gamepad1.left_stick_y;
        double strafeController = gamepad1.left_stick_x;
        double rotationController = gamepad1.right_stick_x;

        forwardController = Range.clip(forwardController, -1, 1);
        strafeController = Range.clip(strafeController, -1, 1);
        rotationController = Range.clip(rotationController, -1, 1);

        drivetrain(forwardController, strafeController, rotationController);
    }

    private void initRobot() {
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");

        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        runtime.reset();
    }

    private void drivetrain(double forward, double strafe, double rotation) {
        frMotor.setPower(forward - strafe - rotation);
        flMotor.setPower(forward/1.65 + strafe + rotation);
        brMotor.setPower(forward + strafe - rotation);
        blMotor.setPower(forward/1.65 - strafe + rotation);
    }

}