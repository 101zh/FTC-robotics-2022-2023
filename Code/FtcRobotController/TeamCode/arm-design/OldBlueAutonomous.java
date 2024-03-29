package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BlueCenterAuto", group = "")
public class BlueCenterAuto extends LinearOpMode {
  private DcMotor raw_shoulder;
  private DcMotor raw_elbow;
  private DcMotor BackRight;
  private DcMotor BackLeft;
  private DcMotor FrontLeft;
  private DcMotor FrontRight;
  private Servo raw_claw;
  private Servo raw_wrist;

  private MyMotor shoulder;
  private MyMotor elbow;
  
  String armStage;
  boolean first;
  int s = 0;
  int e = -7;
  
  /*private enum ArmStatus {
    ARM_STOPPED,
    ARM_UP_STEP_1,
    ARM_UP_STEP_2,
    ARM_UP_STEP_3,
    ARM_UP_STEP_4,
    ARM_UP_STEP_5,
    ARM_UP_STEP_6,
    ARM_DOWN_STEP_1,
    ARM_DOWN_STEP_2,
    DROP_CONE_STEP_1,
    DROP_CONE_STEP_2,
    MOVE_F_1;
  };
  private ArmStatus arm_state = ArmStatus.ARM_STOPPED;
  
  */private float set_servo_pos(float degree) {
        float x = (degree/300);
        return x;
  }/*
  private void arm_up() {
    if (!shoulder.is_running() && !elbow.is_running()) {
      arm_state = ArmStatus.ARM_UP_STEP_2;
    }
  }
  private void arm_up_back() {
    if (!shoulder.is_running() && !elbow.is_running()) {
      arm_state = ArmStatus.ARM_UP_STEP_5;
    }
  }
  private void move_forward() {
    if (!shoulder.is_running() && !elbow.is_running()) {
      arm_state = ArmStatus.MOVE_F_1;
    }
  }
  private void drop_cone() {
    if (!shoulder.is_running() && !elbow.is_running()) {
      arm_state = ArmStatus.DROP_CONE_STEP_1;
    }
  }
  private void arm_down_bottom() {
    if (!shoulder.is_running() && !elbow.is_running()) {
      arm_state = ArmStatus.ARM_UP_STEP_4;
    }
  }
  private void arm_up_top() {
    if (!shoulder.is_running() && !elbow.is_running()) {
      arm_state = ArmStatus.ARM_UP_STEP_2;
    }
  }
  private void handle_arm_state() {
    if (!shoulder.is_running() && !elbow.is_running()) {
      switch(arm_state) {
        case ARM_STOPPED:
          // do nothing
          break;
        case ARM_UP_STEP_1:
          elbow.set_target_pos(20+e);
          first = false;
          arm_state = ArmStatus.ARM_UP_STEP_2;
          break;
        case ARM_UP_STEP_2: //drop fwrd
          shoulder.set_target_pos(-250+s);
          elbow.set_target_pos(185+e);
          armStage = "frwd";
          arm_state = ArmStatus.ARM_UP_STEP_6;
          break;
        case ARM_UP_STEP_3:
          break;
        case ARM_UP_STEP_4: //pickup
          raw_claw.setPosition(.25);
          shoulder.set_target_pos(-130+s);
          elbow.set_target_pos(210+e);
          arm_state = ArmStatus.ARM_STOPPED;
          break;
        case ARM_UP_STEP_5: //drop bkwd
          shoulder.set_target_pos(-230+s);
          elbow.set_target_pos(105+e);
          armStage = "bkwd";
          arm_state = ArmStatus.ARM_STOPPED;
          break;
        case ARM_UP_STEP_6:
            double BackPower = 0.5;
            double BackDuration = 0.5;
            ElapsedTime f;
          
            f = new ElapsedTime();
            while (f.milliseconds() < BackDuration) {
            FrontLeft.setPower(-BackPower);
            FrontRight.setPower(BackPower);
            BackLeft.setPower(-BackPower);
            BackRight.setPower(BackPower);
            }
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
            arm_state = ArmStatus.ARM_STOPPED;
            break;
        case DROP_CONE_STEP_1:
          if (armStage == "frwd") {
            elbow.set_target_pos(210+e);
            shoulder.set_target_pos(-270+s);
          }
          if (armStage == "bkwd") {
            elbow.set_target_pos(65+e);
          }
          arm_state = ArmStatus.DROP_CONE_STEP_2;
          break;
        case DROP_CONE_STEP_2:
          raw_claw.setPosition(.25);
          sleep(1000);
          arm_state = ArmStatus.ARM_STOPPED;
          break;
      }        
    }
  }*/
  // looking from right side, ccw is positive
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
    double Speed;
    double Turn;
    double Strafe;
    double FrontLeft2;
    double FrontRight2;
    double BackLeft2;
    double BackRight2;
    
    
    
  @Override
  public void runOpMode() {
    raw_shoulder = hardwareMap.get(DcMotor.class, "shoulder");
    raw_elbow = hardwareMap.get(DcMotor.class, "elbow");
    raw_claw = hardwareMap.get(Servo.class, "claw");
    raw_wrist = hardwareMap.get(Servo.class, "wrist");
    FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
    FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
    BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
    BackRight = hardwareMap.get(DcMotor.class, "BackRight");
    
    first = true;
    int clawMode = 0;
    int wristMode = 0;
    int clawLoop = 0;
    int wristLoop = 0;
    
    // Reverse one of the drive motors.
    raw_shoulder.setPower(0);
    raw_elbow.setPower(0);
    //raw_wrist.setPosition(.7);
    //raw_claw.setPosition(.25);
    shoulder = new MyMotor(raw_shoulder, -3350, 90, "raw_shoulder");
    elbow = new MyMotor(raw_elbow, -5000, 180, "raw_elbow");
    
    raw_elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    raw_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    raw_elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    raw_shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    raw_elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    raw_shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    
    raw_claw.setPosition(0);
    double BackPower = 0.35;
    double BackDuration = 0.5;
    
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      //raw_wrist.setPosition(.1);
        
        float pos = set_servo_pos(elbow.get_current_degree()+shoulder.get_current_degree()+100);
        
        telemetry.addData("testshoulder+", shoulder.get_current_postion());
        telemetry.addData("testshoulder-", shoulder.get_target_pos());
        telemetry.addData("testelbow+", elbow.get_current_postion());
        telemetry.addData("testelbow-", elbow.get_target_pos());
        //telemetry.addData("arm pos", arm_state);
        telemetry.update();
        
        //shoulder.proceed();
        //elbow.proceed();
        //handle_arm_state();
        
        shoulder.goto_pos_degree(-50+s);
        elbow.goto_pos_degree(-5+e);
        
        sleep(1000);
        raw_wrist.setPosition(pos);
        
        sleep(1000);
        raw_claw.setPosition(.25);
        sleep(1000);
        ElapsedTime f;
        
        
        
        f = new ElapsedTime();//right
        while (f.milliseconds() < 3000) {
            FrontLeft.setPower(BackPower);
            FrontRight.setPower(-BackPower);
            BackLeft.setPower(-BackPower);
            BackRight.setPower(BackPower);
        }
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
        f = new ElapsedTime();
        while (f.milliseconds() < 500) {
            FrontLeft.setPower(BackPower);
            FrontRight.setPower(-BackPower);
            BackLeft.setPower(BackPower);
            BackRight.setPower(-BackPower);
        }
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
        shoulder.goto_pos_degree(0);
        elbow.goto_pos_degree(5);
    }
  }
}
