#include <math.h>
#include "WPILib.h"
#include "ChairBot.h"

#define joystick_null_zone 0.06

#define master_power_factor 1.0

#define smoothing_x_prev_factor 49.0
#define smoothing_x_curr_factor 1.0
#define smoothing_x_factor_total (smoothing_x_prev_factor + smoothing_x_curr_factor)

#define smoothing_y_prev_factor 49.0
#define smoothing_y_curr_factor 1.0
#define smoothing_y_factor_total (smoothing_y_prev_factor + smoothing_y_curr_factor)

#define smoothing_z_prev_factor 49.0
#define smoothing_z_curr_factor 1.0
#define smoothing_z_factor_total (smoothing_z_prev_factor + smoothing_z_curr_factor)

#define deadzone_pots 5
#define pot_max_y 100
#define pot_min_y 0
#define pot_max_x 700
#define pot_min_x 30

ChairBot::ChairBot(void):
	myRobot(dt_pwm_front_left, dt_pwm_rear_left, dt_pwm_front_right, dt_pwm_rear_right),
	stick_s(j_joystick),
	stick_ea(j_3),
	stick_eb(j_4),
	pot_x(an_joystick_x),
	pot_y(an_joystick_y),
	pot_s(an_joystick_pot),
	btn_trig(dg_input_trig),
	btn_top(dg_input_top)
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
}


/*
 * Gets the values for all of the buttons on a joystick and stores them in-place in a vector.
 */
void ChairBot::SetJoystickButtonValueRegister(Joystick *joystick, vector<bool> *value_registry)
{
	/*
	 * Clear out the given vector.
	 *
	 * Since it is passed as a pointer, we don't have to worry about returning anything, since
	 * clearing and re-setting is done in-place.
	 */
	value_registry->clear();

	/*
	 * Loop through the 12 available channels on the joystick.
	 */
	for(uint32_t i = 0; i < 12; i += 1) {
		/*
		 * Get the value for channel (i + 1) [WPIlib likes numbering starting at 1], and push it onto
		 * the registry.
		 */
		value_registry->push_back(joystick->GetRawButton(i + 1));
	}
}

/*
 * Sets the robot up for the teleoperated mode.
 */
void ChairBot::TeleopInit()
{
	/*
	 * Suppress complaints about lack of updating values often enough.
	 */
	myRobot.SetSafetyEnabled(false);

	/*
	 * Get the initial values of the potentiometers on the joysticks.
	 */
	init_pot_x = pot_x.GetValue();
	init_pot_y = pot_y.GetValue();
}

/*
 * Makes the Robot Move and Stuff. We're using Arcade Drive.
 */
void ChairBot::TeleopPeriodic()
{
	/*
	 * Grab values from all of the joysticks as the raw values.
	 */
	SetJoystickButtonValueRegister( &stick_s,  &s_values);
	SetJoystickButtonValueRegister( &stick_ea,  &ea_values);
	SetJoystickButtonValueRegister( &stick_eb,  &eb_values);

	if(eb_values[0x4]) {
		/*
		 * Get the raw joystick axis values from the
		 * wireless/override stick.
		 */
		s_x_raw = stick_s.GetX();
		s_y_raw = stick_s.GetY();
		s_z_raw = stick_s.GetZ();

		/*
		 * Use button 1 as the stick's trigger.
		 */
		bool s_t = s_values[0x0];

		/*
		 * If the stick's trigger is pressed, override raw values with zero.
		 */
		if(s_t) {
			s_x_raw = 0.0;
			s_y_raw = 0.0;
			s_z_raw = 0.0;
		}
	} else {
		/*
		 * Get the raw pot values and then turns it into a raw x value
		 * from 1.0 to -1.0 exclusive, per WPIlib's AnalogChannel.GetValue function.
		 */
		val_pot_x = pot_x.GetValue();
		val_pot_y = pot_y.GetValue();

		/*
		 * Determine if the values are in the deadzones.
		 */
		if(abs(val_pot_x - init_pot_x) < deadzone_pots) {
			s_x_raw = 0;
		} else {
			s_x_raw = 2 * (val_pot_x - init_pot_x) / (pot_max_x - pot_min_x);
		}
		
		if(abs(val_pot_y - init_pot_y) < deadzone_pots) {
			s_y_raw = 0;
		} else {
			s_y_raw = 2 * (val_pot_y - init_pot_y) / (pot_max_y - pot_min_y);
		}

		if(!(bool)btn_trig.Get()) {
			s_x_raw = 0.0;
			s_y_raw = 0.0;
		}
	}
	
	/*
	 * Get AnalogInput #2 from the driver station for future use to limit speed.
	 */
	drive_speed_ain_value = ds->GetAnalogIn(2);

	/*
	 * Apply weighting factors and alleviate the garbage that the joysticks
	 * output when resting (the phantom values).
	 */
	s_x = (((s_x_raw >  joystick_null_zone) || (s_x_raw < - joystick_null_zone)) ? s_x_raw : 0.0);
	s_y = (((s_y_raw >  joystick_null_zone) || (s_y_raw < - joystick_null_zone)) ? s_y_raw : 0.0);
	s_z = (((s_z_raw >  joystick_null_zone) || (s_z_raw < - joystick_null_zone)) ? s_z_raw : 0.0);

	s_x *= (master_power_factor * (drive_speed_ain_value / 5.0));
	s_y *= (master_power_factor * (drive_speed_ain_value / 5.0));
	s_z *= (master_power_factor * (drive_speed_ain_value / 5.0));

	/*
	 * If button 11 on the external joystick is depressed, use jerky movements.
	 * Otherwise, smooth using a weighted average function.
	 */
	if(s_values[0xa]) {
		s_x = s_x;
		s_y = s_y;
		s_z = s_z;
	} else {
		s_x = (((s_x_prev * smoothing_x_prev_factor) + (s_x * smoothing_x_curr_factor)) / (smoothing_x_factor_total));
		s_y = (((s_y_prev * smoothing_y_prev_factor) + (s_y * smoothing_y_curr_factor)) / (smoothing_y_factor_total));
		s_z = (((s_z_prev * smoothing_z_prev_factor) + (s_z * smoothing_z_curr_factor)) / (smoothing_z_factor_total));
	}

	{
		/*
		 * Print out the Joystick values onto the User Messages screen.
		 */
		b->Clear();
		b->Printf(b->kUser_Line1, 1, "x%0.4f", s_x);
		b->Printf(b->kUser_Line2, 1, "y%0.4f", s_y);
		b->Printf(b->kUser_Line3, 1, "z%0.4f", s_z);
		b->Printf(b->kUser_Line4, 1, "s %s%s%s%s%s%s%s%s%s%s%s%s",
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
		b->Printf(b->kUser_Line5, 1, "trig: %d top: %d",
				  btn_trig.Get(),
				  btn_top.Get());
		b->Printf(b->kUser_Line6, 1, "x: %d y: %d z: %d ",
		          pot_x.GetValue(),
		          pot_y.GetValue(),
		          pot_s.GetValue());
		b->UpdateLCD();
	}

	/*
	 * Input the values into the drive function.
	 */
	myRobot.ArcadeDrive(s_y, s_x, s_values[0xb]);

	s_x_prev = s_x;
	s_y_prev = s_y;
	s_z_prev = s_z;
}

START_ROBOT_CLASS(ChairBot);
