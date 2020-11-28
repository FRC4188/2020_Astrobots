
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Auto", group="Linear Opmode")

public class Auto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frMotor, flMotor, brMotor, blMotor;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while(opModeIsActive()) {

            if (runtime.seconds() < 5.0) {
                drivetrain(-1.0, 0, 0 );
            }
            else {
                drivetrain(0,0,0);
            }


        }
    }

    private void initialize() {
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
        frMotor.setPower(forward*1.05 - strafe - rotation);
        flMotor.setPower(forward/1.67 + strafe + rotation);
        brMotor.setPower(forward*1.05 + strafe - rotation);
        blMotor.setPower(forward/1.67 - strafe + rotation);
    }

}
