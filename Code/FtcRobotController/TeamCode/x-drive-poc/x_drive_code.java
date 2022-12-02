package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "x_drive_code")
public class x_drive_code extends LinearOpMode {

  private DcMotor left_back_drive;
  private DcMotor left_front_drive;
  private DcMotor right_front_drive;
  private DcMotor right_back_drive;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    boolean movement;
    double speed;

    left_back_drive = hardwareMap.get(DcMotor.class, "left_back_drive");
    left_front_drive = hardwareMap.get(DcMotor.class, "left_front_drive");
    right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
    right_back_drive = hardwareMap.get(DcMotor.class, "right_back_drive");

    // reversing two of the motors
    left_back_drive.setDirection(DcMotorSimple.Direction.REVERSE);
    left_front_drive.setDirection(DcMotorSimple.Direction.REVERSE);
    movement = false;
    speed = 0.2;
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        // basic movement
        if (gamepad1.dpad_up) {
          speed += 0.05;
          left_front_drive.setPower(speed);
          right_front_drive.setPower(speed);
          left_back_drive.setPower(speed);
          right_back_drive.setPower(speed);
          movement = true;
        }
        else if (gamepad1.dpad_down) {
          speed += 0.05;
          left_front_drive.setPower(-speed);
          right_front_drive.setPower(-speed);
          left_back_drive.setPower(-speed);
          right_back_drive.setPower(-speed);
          movement = true;
        }
        else if (gamepad1.dpad_left) {
          speed += 0.05;
          left_front_drive.setPower(-speed);
          right_front_drive.setPower(speed);
          left_back_drive.setPower(speed);
          right_back_drive.setPower(-speed);
          movement = true;
        }
        else if (gamepad1.dpad_right) {
          speed += 0.05;
          left_front_drive.setPower(speed);
          right_front_drive.setPower(-speed);
          left_back_drive.setPower(-speed);
          right_back_drive.setPower(speed);
          movement = true;
        }
        else if (gamepad1.right_bumper) {
          speed += 0.05;
          left_front_drive.setPower(speed);
          right_front_drive.setPower(-speed);
          left_back_drive.setPower(speed);
          right_back_drive.setPower(-speed);
          movement = true;
        }
        else if (gamepad1.left_bumper) {
          speed += 0.05;
          left_front_drive.setPower(-speed);
          right_front_drive.setPower(speed);
          left_back_drive.setPower(-speed);
          right_back_drive.setPower(speed);
          movement = true;
        }
        else if (movement == true) {
          speed = 0;
          left_back_drive.setPower(speed);
          left_front_drive.setPower(speed);
          right_back_drive.setPower(speed);
          right_front_drive.setPower(speed);
          movement = false;
        }
      }
    }
  }
}
