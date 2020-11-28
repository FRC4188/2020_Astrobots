package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "SquareOp")
public class SquareOp extends LinearOpMode {

    private DcMotor frMotor, flMotor, brMotor, blMotor;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        frMotor = hardwareMap.dcMotor.get("frMotor");
        flMotor = hardwareMap.dcMotor.get("flMotor");
        brMotor = hardwareMap.dcMotor.get("brMotor");
        blMotor = hardwareMap.dcMotor.get("blMotor");

        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            double time = runtime.seconds();

            telemetry.addData("time", time);
            telemetry.update();


           if (time < 1) {
                drivetrain(1, 0, 0);
            }

            if (time > 1 && time < 2) {
                drivetrain(0, 1, 0);
            }

            if (time > 2 && time < 3) {
                drivetrain(-1, 0, 0);
            }

            if (time > 3 && time < 4) {
                drivetrain(0, -1, 0);
            }

            if (time > 4 && time < 5) {
                drivetrain(1, 0, 0);
            }

            if(time > 5)
                drivetrain(0, 0,0);
        }

        drivetrain(0, 0,0);
    }

    private void drivetrain(double forward, double strafe, double rotation) {
        double frPower = forward - strafe - rotation;
        double flPower = forward + strafe + rotation;
        double brPower = forward + strafe - rotation;
        double blPower = forward - strafe + rotation;

        double maxPower = Math.max(Math.abs(frPower), Math.abs(flPower));
        maxPower = Math.max(maxPower, Math.abs(brPower));
        maxPower = Math.max(maxPower, Math.abs(blPower));

        frMotor.setPower(frPower / maxPower);
        flMotor.setPower(flPower / maxPower);
        brMotor.setPower(brPower / maxPower);
        blMotor.setPower(blPower / maxPower);
    }
}