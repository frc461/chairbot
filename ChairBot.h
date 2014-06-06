#include <math.h>
#include "WPILIB.h"

#ifndef CHAIRBOT_H
#define CHAIRBOT_H

enum RobotDriveTrainPorts {
	dt_pwm_rear_right = 1,
	dt_pwm_rear_left = 2,
	dt_pwm_front_right = 3,
	dt_pwm_front_left = 4
};

enum RobotMiscellaneousPorts {
	m_rel_aim_confirm_led = 2
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

class ChairBot : public IterativeRobot
{
	RobotDrive myRobot;
	Joystick stick_s;
	DriverStation *ds;
	DriverStationLCD *b;

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
	ChairBot();

	void SetJoystickButtonValueRegister(Joystick *, vector<bool> *);

	void TeleopInit();
	void TeleopPeriodic();
};
#endif
