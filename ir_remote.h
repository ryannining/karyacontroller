

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

extern void IR_setup();
extern void IR_end();
extern void IR_loop();
extern bool IR_ok;
extern String jobnum;
