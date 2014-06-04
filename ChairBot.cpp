#include <math.h>
#include "WPILib.h"

enum RobotDriveTrainPorts {
	dt_pwm_rear_right = 1,
	dt_pwm_rear_left = 2,
	dt_pwm_front_right = 3,
	dt_pwm_front_left = 4
};

enum RobotMiscellaneousPorts {
	m_rel_aim_confirm_led = 2,
	m_rel_spinny_light = 10
};

enum RobotJoystickPorts {
	j_1 = 1,
	j_2 = 2,
	j_3 = 3,
	j_4 = 4,
	j_5 = 5,
	j_6 = 6,
	j_joystick = j_1
};

enum Attack3Buttons {
	atk3_btn_1 = 0x1,
	atk3_btn_2 = 0x2,
	atk3_btn_3 = 0x3,
	atk3_btn_4 = 0x4,
	atk3_btn_5 = 0x5,
	atk3_btn_6 = 0x6,
	atk3_btn_7 = 0x7,
	atk3_btn_8 = 0x8,
	atk3_btn_9 = 0x9,
	atk3_btn_10 = 0xa,
	atk3_btn_11 = 0xb,
	atk3_btn_12 = 0xc,
	atk3_btn_trigger = atk3_btn_1,
	atk3_btn_thumb_back = atk3_btn_2,
	atk3_btn_thumb_front = atk3_btn_3,
	atk3_btn_thumb_left = atk3_btn_4,
	atk3_btn_thumb_right = atk3_btn_5
};

#define stklport 1
#define stkrport 2
#define stkeaport 3
#define stkebport 4

#define master_js_null_zone 0.06
#define left_js_null_zone master_js_null_zone
#define right_js_null_zone master_js_null_zone

#define master_power_factor 1.0
#define master_trigger_power_factor 0.25

#define dual_triggers_multiply_again false

#define use_mecanum true

class RowdyFifteen : public IterativeRobot
{
	RobotDrive myRobot;
	Joystick stick_s;
	DriverStation *ds;
	DriverStationLCD *b;
	DigitalOutput spinny_light;

	vector<bool> s_values;
	float s_x_raw;
	float s_y_raw;
	float s_z_raw;
	
	float s_x;
	float s_y;
	float s_z;
	
	float s_x_prev;
	float s_y_prev;
	float s_z_prev;
	
	float drive_speed_ain_value;
	
public:
	RowdyFifteen():
		myRobot(dt_pwm_front_left, dt_pwm_rear_left, dt_pwm_front_right, dt_pwm_rear_right),
		stick_s(j_joystick),
		spinny_light(10)
	{
		/*
		 * Configure the joysticks to have the correct channels.
		 */
		stick_s.SetAxisChannel(Joystick::kXAxis, 1);
		stick_s.SetAxisChannel(Joystick::kYAxis, 2);
		stick_s.SetAxisChannel(Joystick::kZAxis, 3);

		/*
		 * Get the raw axis values from the joysticks
		 */
		s_x_raw = stick_s.GetX();
		s_y_raw = stick_s.GetY();
		s_z_raw = stick_s.GetZ();
		
		/*
		 * Get an instance of the Driver Station stuff
		 */
		ds = DriverStation::GetInstance();
		b = DriverStationLCD::GetInstance();
		
		b->Clear();
		b->UpdateLCD();

		/*
		 * Invert the motors.
		 *
		 * This helps Mecanum_Cartesian to work properly.
		 */
		myRobot.SetInvertedMotor(myRobot.kFrontLeftMotor, true);
		myRobot.SetInvertedMotor(myRobot.kFrontRightMotor, true);
		myRobot.SetInvertedMotor(myRobot.kRearLeftMotor, true);
		myRobot.SetInvertedMotor(myRobot.kRearRightMotor, true);
		
		SmartDashboard::init();
		
		spinny_light.Set(1);
	}

	vector<bool> GetJoystickButtonValues(Joystick *joystick)
	{
		vector<bool> values;

		for(uint32_t i = 0; i < 12; i += 1) {
			values.push_back(joystick->GetRawButton(i + 1));
		}

		return values;
	}
	
	void SetJoystickButtonValueRegister(Joystick *joystick, vector<bool> *value_registry)
	{
		value_registry->clear();
		
		for(uint32_t i = 0; i < 12; i += 1) {
			value_registry->push_back(joystick->GetRawButton(i + 1));
		}
	}
	
	void UpdateSmartDashboard()
	{
	}

	/*
	 * Drive left & right motors for 2 seconds then stop
	 */
	void AutonomousInit()
	{
		spinny_light.Set(1);
	}
	
	void AutonomousPeriodic()
	{
		UpdateSmartDashboard();
		spinny_light.Set(1);
	}

	void TeleopInit()
	{
		myRobot.SetSafetyEnabled(false);
		spinny_light.Set(1);
	}
	
	/*
	 * Runs the motors with mecanum.
	 *
	 * NOTE: Broken
	 */
	void TeleopPeriodic()
	{
		/*
		 * Grab values from all of the joysticks as the raw values.
		 */
		s_x_raw = stick_s.GetX();
		s_y_raw = stick_s.GetY();
		s_z_raw = stick_s.GetZ();
		
		SetJoystickButtonValueRegister( &stick_s,  &s_values);
		
		/*
		 * Get the trigger values.
		 */
		bool s_t = s_values[0x0];
		
		if(s_t) {
			s_x_raw = 0.0;
			s_y_raw = 0.0;
			s_z_raw = 0.0;
		}

		drive_speed_ain_value = ds->GetAnalogIn(2);
		
		/*
		 * Apply weighting factors and alleviate the garbage that the joysticks
		 * output when resting (the phantom values).
		 */
		s_x = (((s_x_raw >  left_js_null_zone) || (s_x_raw < - left_js_null_zone)) ? s_x_raw : 0.0);
		s_y = (((s_y_raw >  left_js_null_zone) || (s_y_raw < - left_js_null_zone)) ? s_y_raw : 0.0);
		s_z = (((s_z_raw >  left_js_null_zone) || (s_z_raw < - left_js_null_zone)) ? s_z_raw : 0.0);
		
		s_x *= master_power_factor;
		s_y *= master_power_factor;
		s_z *= master_power_factor;

		s_x *= (drive_speed_ain_value / 5.0);
		s_y *= (drive_speed_ain_value / 5.0);
		s_z *= (drive_speed_ain_value / 5.0);
		
		if(s_values[0xa]) {
			s_x = s_x;
			s_y = s_y;
			s_z = s_z;
		} else {
			s_x = (((s_x_prev * 49.0) + (s_x * 1.0)) / (49.0 + 1.0));
			s_y = (((s_y_prev * 49.0) + (s_y * 1.0)) / (49.0 + 1.0));
			s_z = (((s_z_prev * 49.0) + (s_z * 1.0)) / (49.0 + 1.0));
		}
		
		{
			/*
			 * Print out the Joystick values onto the User Messages screen.
			 */
			b->Clear();
			b->Printf(b->kUser_Line2, 1, "x%0.4f", s_x);
			b->Printf(b->kUser_Line3, 1, "y%0.4f", s_y);
			b->Printf(b->kUser_Line4, 1, "z%0.4f", s_z);
			b->Printf(b->kUser_Line6, 1, "s %s%s%s%s%s%s%s%s%s%s%s%s",
					(s_values[0x0] ? "1" : ""),
					(s_values[0x1] ? "2" : ""),
					(s_values[0x2] ? "3" : ""),
					(s_values[0x3] ? "4" : ""),
					(s_values[0x4] ? "5" : ""),
					(s_values[0x5] ? "6" : ""),
					(s_values[0x6] ? "7" : ""),
					(s_values[0x7] ? "8" : ""),
					(s_values[0x8] ? "9" : ""),
					(s_values[0x9] ? "a" : ""),
					(s_values[0xa] ? "b" : ""),
					(s_values[0xb] ? "c" : ""));
			b->UpdateLCD();
		}
		
		UpdateSmartDashboard();
		
		/*
		 * Input the values into the drive function.
		 */
		myRobot.ArcadeDrive(s_y, s_x, s_values[0xb]);
		
		spinny_light.Set(1);
		
		s_x_prev = s_x;
		s_y_prev = s_y;
		s_z_prev = s_z;
	}

	/*
	 * Runs during test mode
	 */
	void TestInit()
	{
		/*
		 * Print out a cute little message to let people know that we're in test mode.
		 */
		b->Clear();
		b->Printf(b->kUser_Line1, 1, "TEST mode!");
		b->UpdateLCD();
	}
	
	void TestPeriodic()
	{
		UpdateSmartDashboard();
	}
	
	void DisabledInit()
	{
	}
	
	void DisabledPeriodic()
	{	
		SetJoystickButtonValueRegister( &stick_s,  &s_values);
		
		b->Clear();
		b->Printf(b->kUser_Line1, 1, "Robot is disabled!");
		b->Printf(b->kUser_Line6, 1, "s %s%s%s%s%s%s%s%s%s%s%s%s",
				(s_values[0x0] ? "1" : ""),
				(s_values[0x1] ? "2" : ""),
				(s_values[0x2] ? "3" : ""),
				(s_values[0x3] ? "4" : ""),
				(s_values[0x4] ? "5" : ""),
				(s_values[0x5] ? "6" : ""),
				(s_values[0x6] ? "7" : ""),
				(s_values[0x7] ? "8" : ""),
				(s_values[0x8] ? "9" : ""),
				(s_values[0x9] ? "a" : ""),
				(s_values[0xa] ? "b" : ""),
				(s_values[0xb] ? "c" : ""));
		b->UpdateLCD();
		
		UpdateSmartDashboard();
		
		Wait(0.05);
	}
};

START_ROBOT_CLASS(RowdyFifteen);
