#ifndef HX_30HM_DEF_H
#define HX_30HM_DEF_H

/* nvs */
#define REG_SERVO_MAIN_VERSION	3
#define REG_SERVO_SEC_VERSION	4
#define REG_ID					5
#define REG_BAUD_RATE			6
#define REG_CW_DEAD				26
#define REG_CCW_DEAD			27
#define REG_POS_OFFSET_L		31
#define REG_POS_OFFSET_H		32
#define REG_MODE				33

/* read-write */
#define REG_TORQUE_ENABLE		40
#define REG_ACC					41
#define REG_GOAL_POSITION_L		42
#define REG_GOAL_POSITION_H		43
#define REG_PWM_SPEED_L			44
#define REG_PWM_SPEED_H			45
#define REG_GOAL_SPEED_L		46
#define REG_GOAL_SPEED_H		47
#define REG_MAX_TORQUE_L		48
#define REG_MAX_TORQUE_H		49

/* only read */
#define REG_PRESENT_POSITION_L	56
#define REG_PRESENT_POSITION_H	57
#define REG_PRESENT_SPEED_L		58
#define REG_PRESENT_SPEED_H		59
#define REG_PRESENT_LOAD_L		60
#define REG_PRESENT_LOAD_H		61
#define REG_PRESENT_VOLTAGE		62
#define REG_PRESENT_TEMPERATURE	63
#define REG_MOVING_STATUS		66
#define REG_PRESENT_CURRENT_L	69
#define REG_PRESENT_CURRENT_H	70

#define FRAME_HEADER_1			0xFF
#define FRAME_HEADER_2			0xFF

#define CMD_PING				1
#define CMD_READ				2
#define CMD_WRITE				3
#define CMD_REG_WRITE			4
#define CMD_ACTION				5
#define CMD_RESET				6
#define CMD_SYNC_READ			130
#define CMD_SYNC_WRITE 			131

#define BROADCAST_ID			0xFE

#define BAUD_RATE_1M			0
#define BAUD_RATE_0_5_M			1
#define BAUD_RATE_250K			2
#define BAUD_RATE_115200		4
#define BAUD_RATE_76800			5
#define BAUD_RATE_57600			6
#define BAUD_RATE_38400			7

#define POSITION_MODE			0
#define CLOSED_LOOP_MOTOR_MODE	1
#define OPEN_LOOP_MOTOR_MODE 	2

#endif