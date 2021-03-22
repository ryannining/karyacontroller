

#define  IRK_1 0x45
#define  IRK_2 0x46
#define  IRK_3 0x47
#define  IRK_4 0x44
#define  IRK_5 0x40
#define  IRK_6 0x43
#define  IRK_7 0x7
#define  IRK_8 0x15
#define  IRK_9 0x9
#define  IRK_0 0x19
#define  IRK_X 0x16
#define  IRK_H 0xD
#define  IRK_UP 0x18
#define  IRK_LF 0x8
#define  IRK_RG 0x5A
#define  IRK_DN 0x52
#define  IRK_OK 0x1C


#define RM4(n) (n + 1024)
#define  IRK4_OK RM4(68)
#define  IRK4_PAUSE RM4(186)
#define  IRK4_PLAY RM4(176)
#define  IRK4_SLOWDOWN RM4(143)
#define  IRK4_FASTER RM4(142)
#define  IRK4_JOBINDEX RM4(112)
#define  IRK4_EXIT RM4(91)
#define  IRK4_SETTINGS RM4(67)
#define  IRK4_ZUP RM4(0)
#define  IRK4_ZDN RM4(1)
#define  IRK4_SPINDLEUP RM4(2)
#define  IRK4_SPINDLEDN RM4(3)
#define  IRK4_BACK2 RM4(40)
#define  IRK4_BACK RM4(7)
#define  IRK4_UP RM4(64)
#define  IRK4_LF RM4(7)
#define  IRK4_RG RM4(6)
#define  IRK4_DN RM4(65)
#define  IRK4_X RM4(83)
#define  IRK4_H RM4(26)
#define  IRK4_1 RM4(17)
#define  IRK4_2 RM4(18)
#define  IRK4_3 RM4(19)
#define  IRK4_4 RM4(20)
#define  IRK4_5 RM4(21)
#define  IRK4_6 RM4(22)
#define  IRK4_7 RM4(23)
#define  IRK4_8 RM4(24)
#define  IRK4_9 RM4(25)
#define  IRK4_0 RM4(16)
#define  IRK4_STOP RM4(177)
#define  IRK4_POWER RM4(8)
#define  IRK4_DOT1 1138
#define  IRK4_DOT2 1137
#define  IRK4_DOT3 1123
#define  IRK4_DOT4 1121




extern void IR_setup();
extern void IR_end();
extern void IR_loop(int mode);
extern bool IR_ok;
extern String jobnum;
