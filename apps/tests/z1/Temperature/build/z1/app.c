#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 150 "/opt/local/lib/gcc/msp430/4.7.0/include/stddef.h" 3
typedef int ptrdiff_t;
#line 213
typedef unsigned int size_t;
#line 325
typedef int wchar_t;
# 8 "/Users/Xavier/Documents/tinyOS/nesc/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
#line 8
  int dummy;
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
#line 13
  int dummy;
}  ;
#line 14
struct __nesc_attr_one_nok {
#line 14
  int dummy;
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
#line 17
  int dummy;
}  ;
# 38 "/opt/local/lib/gcc/msp430/4.7.0/../../../../msp430/include/stdint.h" 3
typedef signed char int8_t;
typedef int int16_t;
typedef long int int32_t;
__extension__ 
#line 41
typedef long long int int64_t;

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long int uint32_t;
__extension__ 
#line 46
typedef unsigned long long int uint64_t;


typedef long int __attribute((__a20__)) int20_t;
typedef unsigned long int __attribute((__a20__)) uint20_t;






typedef signed char int_least8_t;
typedef int int_least16_t;
typedef long int int_least32_t;
__extension__ 
#line 60
typedef long long int int_least64_t;


typedef unsigned char uint_least8_t;
typedef unsigned int uint_least16_t;
typedef unsigned long int uint_least32_t;
__extension__ 
#line 66
typedef unsigned long long int uint_least64_t;





typedef signed char int_fast8_t;
typedef int int_fast16_t;
typedef long int int_fast32_t;
__extension__ 
#line 75
typedef long long int int_fast64_t;


typedef unsigned char uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned long int uint_fast32_t;
__extension__ 
#line 81
typedef unsigned long long int uint_fast64_t;









typedef int16_t intptr_t;
typedef uint16_t uintptr_t;




__extension__ 
#line 97
typedef long long int intmax_t;
__extension__ 
#line 98
typedef unsigned long long int uintmax_t;
# 431 "/Users/Xavier/Documents/tinyOS/nesc/lib/ncc/nesc_nx.h"
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 48 "/opt/local/lib/gcc/msp430/4.7.0/../../../../msp430/include/sys/types.h" 3
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 44 "/opt/local/lib/gcc/msp430/4.7.0/../../../../msp430/include/string.h" 3
extern void *memset(void *arg_0x10b2015d8, int arg_0x10b201840, size_t arg_0x10b201ae8);
#line 65
extern void *memset(void *arg_0x10b21c868, int arg_0x10b21cad0, size_t arg_0x10b21cd78);
# 62 "/opt/local/lib/gcc/msp430/4.7.0/../../../../msp430/include/stdlib.h" 3
#line 59
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;






#line 66
typedef struct __nesc_unnamed4243 {
  long int quot;
  long int rem;
} ldiv_t;
# 122 "/opt/local/lib/gcc/msp430/4.7.0/../../../../msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/opt/local/lib/gcc/msp430/4.7.0/../../../../msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
typedef unsigned long _useconds_t;
typedef long _suseconds_t;
typedef long _time_t;
typedef long _clock_t;
typedef int _clockid_t;
# 19 "/opt/local/lib/gcc/msp430/4.7.0/../../../../msp430/include/sys/reent.h" 3
typedef unsigned long __ULong;
#line 31
struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *arg_0x10b255e08);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x10b259020);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/opt/local/lib/gcc/msp430/4.7.0/../../../../msp430/include/math.h" 3
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 212
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 265
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 25 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 26
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;







struct __nesc_attr_atmostonce {
};
#line 37
struct __nesc_attr_atleastonce {
};
#line 38
struct __nesc_attr_exactlyonce {
};
# 51 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/types/TinyError.h"
enum __nesc_unnamed4248 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2)  ;
# 47 "/opt/local/lib/gcc/msp430/4.7.0/../../../../msp430/include/intrinsics.h" 3
void __nop(void );



void __dint(void );



void __eint(void );


unsigned int __read_status_register(void );


typedef unsigned int __istate_t;
# 168 "/opt/local/lib/gcc/msp430/4.7.0/../../../../msp430/include/msp430f2617.h" 3
extern volatile __attribute((__d16__)) unsigned char IFG2 __asm ("__""IFG2");







extern volatile __attribute((__d16__)) unsigned char UC1IE __asm ("__""UC1IE");






extern volatile __attribute((__d16__)) unsigned char UC1IFG __asm ("__""UC1IFG");
#line 195
extern volatile __attribute((__d16__)) unsigned int ADC12CTL0 __asm ("__""ADC12CTL0");

extern volatile __attribute((__d16__)) unsigned int ADC12CTL1 __asm ("__""ADC12CTL1");
#line 454
extern volatile __attribute((__d16__)) unsigned char DCOCTL __asm ("__""DCOCTL");

extern volatile __attribute((__d16__)) unsigned char BCSCTL1 __asm ("__""BCSCTL1");
#line 923
extern volatile __attribute((__d16__)) unsigned char P3SEL __asm ("__""P3SEL");
#line 947
extern volatile __attribute((__d16__)) unsigned char P5OUT __asm ("__""P5OUT");

extern volatile __attribute((__d16__)) unsigned char P5DIR __asm ("__""P5DIR");



extern volatile __attribute((__d16__)) unsigned char P5REN __asm ("__""P5REN");
#line 1035
extern volatile __attribute((__d16__)) unsigned int TACTL __asm ("__""TACTL");

extern volatile __attribute((__d16__)) unsigned int TACCTL0 __asm ("__""TACCTL0");

extern volatile __attribute((__d16__)) unsigned int TACCTL1 __asm ("__""TACCTL1");

extern volatile __attribute((__d16__)) unsigned int TACCTL2 __asm ("__""TACCTL2");

extern volatile __attribute((__d16__)) unsigned int TAR __asm ("__""TAR");





extern volatile __attribute((__d16__)) unsigned int TACCR2 __asm ("__""TACCR2");
#line 1174
extern volatile __attribute((__d16__)) unsigned int TBR __asm ("__""TBR");
#line 1293
extern volatile __attribute((__d16__)) unsigned char UCA0CTL1 __asm ("__""UCA0CTL1");

extern volatile __attribute((__d16__)) unsigned char UCA0BR0 __asm ("__""UCA0BR0");

extern volatile __attribute((__d16__)) unsigned char UCA0BR1 __asm ("__""UCA0BR1");

extern volatile __attribute((__d16__)) unsigned char UCA0MCTL __asm ("__""UCA0MCTL");





extern volatile __attribute((__d16__)) unsigned char UCA0TXBUF __asm ("__""UCA0TXBUF");
#line 1318
extern volatile __attribute((__d16__)) unsigned char UCB0CTL1 __asm ("__""UCB0CTL1");
#line 1340
extern volatile __attribute((__d16__)) unsigned char UCA1CTL1 __asm ("__""UCA1CTL1");









extern const volatile __attribute((__d16__)) unsigned char UCA1RXBUF __asm ("__""UCA1RXBUF");
#line 1363
extern volatile __attribute((__d16__)) unsigned char UCB1CTL0 __asm ("__""UCB1CTL0");

extern volatile __attribute((__d16__)) unsigned char UCB1CTL1 __asm ("__""UCB1CTL1");

extern volatile __attribute((__d16__)) unsigned char UCB1BR0 __asm ("__""UCB1BR0");

extern volatile __attribute((__d16__)) unsigned char UCB1BR1 __asm ("__""UCB1BR1");



extern volatile __attribute((__d16__)) unsigned char UCB1STAT __asm ("__""UCB1STAT");

extern const volatile __attribute((__d16__)) unsigned char UCB1RXBUF __asm ("__""UCB1RXBUF");





extern volatile __attribute((__d16__)) unsigned int UCB1I2CSA __asm ("__""UCB1I2CSA");
#line 1556
extern volatile __attribute((__d16__)) unsigned int WDTCTL __asm ("__""WDTCTL");
#line 1653
extern const volatile __attribute((__d16__)) unsigned char CALDCO_8MHZ __asm ("__""CALDCO_8MHZ");

extern const volatile __attribute((__d16__)) unsigned char CALBC1_8MHZ __asm ("__""CALBC1_8MHZ");
# 389 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/msp430hardware.h"
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)  ;


enum __nesc_unnamed4249 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void )  ;





static inline void __nesc_enable_interrupt(void )  ;




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






__nesc_atomic_t __nesc_atomic_start(void )   ;







void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)   ;
#line 444
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_float;typedef float __nesc_nxbase_nx_float  ;
#line 459
enum __nesc_unnamed4250 {
  MSP430_PORT_RESISTOR_INVALID, 
  MSP430_PORT_RESISTOR_OFF, 
  MSP430_PORT_RESISTOR_PULLDOWN, 
  MSP430_PORT_RESISTOR_PULLUP
};
# 40 "/opt/local/lib/gcc/msp430/4.7.0/include/stdarg.h" 3
typedef __builtin_va_list __gnuc_va_list;
#line 102
typedef __gnuc_va_list va_list;
# 51 "/opt/local/lib/gcc/msp430/4.7.0/../../../../msp430/include/stdio.h" 3
int __attribute((format(printf, 2, 3))) sprintf(char *buf, const char *fmt, ...);
# 87 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/PrintfUART.h"
char debugbuf[256];
#line 126
static inline void printfUART_init_private();
#line 319
static void UARTPutChar(char c);
#line 349
static void writedebug();
# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/platforms/z1/chips/msp430/timer/Msp430XDcoCalib.h"
static inline void Set_DCO(unsigned int Delta);
#line 108
static inline void Set_DCO(unsigned int Delta);
# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4251 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};
#line 75
#line 62
typedef struct __nesc_unnamed4252 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} msp430_compare_control_t;
#line 87
#line 77
typedef struct __nesc_unnamed4253 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} msp430_timer_a_control_t;
#line 102
#line 89
typedef struct __nesc_unnamed4254 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} msp430_timer_b_control_t;
# 41 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4255 {
#line 41
  int notUsed;
} 
#line 41
TSecond;
typedef struct __nesc_unnamed4256 {
#line 42
  int notUsed;
} 
#line 42
TMilli;
typedef struct __nesc_unnamed4257 {
#line 43
  int notUsed;
} 
#line 43
T32khz;
typedef struct __nesc_unnamed4258 {
#line 44
  int notUsed;
} 
#line 44
TMicro;
# 43 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/types/Leds.h"
enum __nesc_unnamed4259 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 40 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/types/I2C.h"
typedef struct __nesc_unnamed4260 {
} 
#line 40
TI2CExtdAddr;
typedef struct __nesc_unnamed4261 {
} 
#line 41
TI2CBasicAddr;

typedef uint8_t i2c_flags_t;

enum __nesc_unnamed4262 {
  I2C_START = 0x01, 
  I2C_STOP = 0x02, 
  I2C_ACK_END = 0x04
};
# 90 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/msp430usci.h"
#line 85
typedef enum __nesc_unnamed4263 {
  USCI_NONE = 0, 
  USCI_UART = 1, 
  USCI_SPI = 2, 
  USCI_I2C = 3
} msp430_uscimode_t;
#line 111
#line 103
typedef struct __nesc_unnamed4264 {
  unsigned int ucsync : 1;
  unsigned int ucmode : 2;
  unsigned int ucspb : 1;
  unsigned int uc7bit : 1;
  unsigned int ucmsb : 1;
  unsigned int ucpar : 1;
  unsigned int ucpen : 1;
} __attribute((packed))  msp430_uctl0_t;
#line 126
#line 118
typedef struct __nesc_unnamed4265 {
  unsigned int ucswrst : 1;
  unsigned int uctxbrk : 1;
  unsigned int uctxaddr : 1;
  unsigned int ucdorm : 1;
  unsigned int ucbrkie : 1;
  unsigned int ucrxeie : 1;
  unsigned int ucssel : 2;
} __attribute((packed))  msp430_uctl1_t;
#line 185
#line 145
typedef enum __nesc_unnamed4266 {
  UBR_32KHZ_1200 = 0x001B, UMCTL_32KHZ_1200 = 0x04, 
  UBR_32KHZ_2400 = 0x000D, UMCTL_32KHZ_2400 = 0x0c, 
  UBR_32KHZ_4800 = 0x0006, UMCTL_32KHZ_4800 = 0x0e, 
  UBR_32KHZ_9600 = 0x0003, UMCTL_32KHZ_9600 = 0x06, 

  UBR_1048MHZ_9600 = 0x006D, UMCTL_1048MHZ_9600 = 0x04, 
  UBR_1048MHZ_19200 = 0x0036, UMCTL_1048MHZ_19200 = 0x0a, 
  UBR_1048MHZ_38400 = 0x001B, UMCTL_1048MHZ_38400 = 0x04, 
  UBR_1048MHZ_57600 = 0x0012, UMCTL_1048MHZ_57600 = 0x0c, 
  UBR_1048MHZ_115200 = 0x0009, UMCTL_1048MHZ_115200 = 0x02, 
  UBR_1048MHZ_128000 = 0x0008, UMCTL_1048MHZ_128000 = 0x02, 
  UBR_1048MHZ_256000 = 0x0004, UMCTL_1048MHZ_230400 = 0x02, 








  UBR_1MHZ_9600 = 0x6, UMCTL_1MHZ_9600 = 0x81, 
  UBR_1MHZ_19200 = 0x3, UMCTL_1MHZ_19200 = 0x41, 
  UBR_1MHZ_57600 = 0x1, UMCTL_1MHZ_57600 = 0x0F, 

  UBR_8MHZ_4800 = 0x68, UMCTL_8MHZ_4800 = 0x31, 
  UBR_8MHZ_9600 = 0x34, UMCTL_8MHZ_9600 = 0x11, 
  UBR_8MHZ_19200 = 0x1A, UMCTL_8MHZ_19200 = 0x11, 
  UBR_8MHZ_38400 = 0x0D, UMCTL_8MHZ_38400 = 0x01, 
  UBR_8MHZ_57600 = 0x08, UMCTL_8MHZ_57600 = 0xB1, 
  UBR_8MHZ_115200 = 0x04, UMCTL_8MHZ_115200 = 0x3B, 
  UBR_8MHZ_230400 = 0x02, UMCTL_8MHZ_230400 = 0x27, 

  UBR_16MHZ_4800 = 0xD0, UMCTL_16MHZ_4800 = 0x51, 
  UBR_16MHZ_9600 = 0x68, UMCTL_16MHZ_9600 = 0x31, 
  UBR_16MHZ_19200 = 0x34, UMCTL_16MHZ_19200 = 0x11, 
  UBR_16MHZ_38400 = 0x1A, UMCTL_16MHZ_38400 = 0x11, 
  UBR_16MHZ_57600 = 0x11, UMCTL_16MHZ_57600 = 0x61, 
  UBR_16MHZ_115200 = 0x8, UMCTL_16MHZ_115200 = 0xB1, 
  UBR_16MHZ_230400 = 0x4, UMCTL_16MHZ_230400 = 0x3B
} msp430_uart_rate_t;
#line 211
#line 188
typedef struct __nesc_unnamed4267 {
  unsigned int ubr : 16;
  unsigned int umctl : 8;


  unsigned int  : 1;
  unsigned int ucmode : 2;
  unsigned int ucspb : 1;
  unsigned int uc7bit : 1;
  unsigned int  : 1;
  unsigned int ucpar : 1;
  unsigned int ucpen : 1;


  unsigned int  : 5;
  unsigned int ucrxeie : 1;
  unsigned int ucssel : 2;




  unsigned int utxe : 1;
  unsigned int urxe : 1;
} msp430_uart_config_t;







#line 213
typedef struct __nesc_unnamed4268 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl0;
  uint8_t uctl1;
  uint8_t ume;
} msp430_uart_registers_t;




#line 221
typedef union __nesc_unnamed4269 {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;
#line 264
#line 248
typedef struct __nesc_unnamed4270 {
  unsigned int ubr : 16;


  unsigned int  : 1;
  unsigned int ucmode : 2;
  unsigned int ucmst : 1;
  unsigned int uc7bit : 1;
  unsigned int ucmsb : 1;
  unsigned int ucckpl : 1;
  unsigned int ucckph : 1;


  unsigned int  : 1;
  unsigned int  : 5;
  unsigned int ucssel : 2;
} msp430_spi_config_t;






#line 267
typedef struct __nesc_unnamed4271 {
  uint16_t ubr;
  uint8_t uctl0;
  uint8_t uctl1;
} msp430_spi_registers_t;




#line 273
typedef union __nesc_unnamed4272 {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;
#line 305
#line 297
typedef struct __nesc_unnamed4273 {
  unsigned int  : 1;
  unsigned int ucmode : 2;
  unsigned int ucmst : 1;
  unsigned int  : 1;
  unsigned int ucmm : 1;
  unsigned int ucsla10 : 1;
  unsigned int uca10 : 1;
} __attribute((packed))  msp430_i2cctl0_t;
#line 320
#line 312
typedef struct __nesc_unnamed4274 {
  unsigned int ucswrst : 1;
  unsigned int uctxstt : 1;
  unsigned int uctxstp : 1;
  unsigned int uctxnack : 1;
  unsigned int uctr : 1;
  unsigned int  : 1;
  unsigned int ucssel : 2;
} __attribute((packed))  msp430_i2cctl1_t;
#line 348
#line 323
typedef struct __nesc_unnamed4275 {
  uint16_t ubr : 16;


  uint8_t  : 1;
  uint8_t ucmode : 2;
  uint8_t ucmst : 1;
  uint8_t  : 1;
  uint8_t ucmm : 1;
  uint8_t ucsla10 : 1;
  uint8_t uca10 : 1;


  uint8_t  : 1;
  uint8_t  : 1;
  uint8_t  : 1;
  uint8_t  : 1;
  uint8_t uctr : 1;
  uint8_t  : 1;
  uint8_t ucssel : 2;


  uint16_t i2coa : 10;
  uint8_t  : 5;
  uint8_t ucgcen : 1;
} msp430_i2c_config_t;






#line 350
typedef struct __nesc_unnamed4276 {
  uint16_t ubr;
  uint8_t uctl0;
  uint8_t uctl1;
  uint16_t ui2coa;
} msp430_i2c_registers_t;




#line 357
typedef union __nesc_unnamed4277 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
# 33 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
typedef TMilli TestTmp102C__TestTimer__precision_tag;
typedef uint16_t TestTmp102C__TempSensor__val_t;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__0____nesc_unnamed4278 {
  Msp430Timer32khzC__0__ALARM_ID = 0U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__size_type;
typedef uint16_t SimpleTMP102P__Read__val_t;
typedef TMilli SimpleTMP102P__TimerSensor__precision_tag;
typedef TMilli SimpleTMP102P__TimerFail__precision_tag;
typedef TI2CBasicAddr SimpleTMP102P__I2CBasicAddr__addr_size;
enum /*TestTmp102AppC.Temperature.I2C*/Msp430I2C1C__0____nesc_unnamed4279 {
  Msp430I2C1C__0__CLIENT_ID = 0U
};
typedef TI2CBasicAddr /*Msp430I2C1P.I2CP*/Msp430I2CP__0__I2CBasicAddr__addr_size;
enum /*TestTmp102AppC.Temperature.I2C.UsciC*/Msp430UsciB1C__0____nesc_unnamed4280 {
  Msp430UsciB1C__0__CLIENT_ID = 1U
};
# 62 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
# 46 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 43
static void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );



static void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 42
static void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );





static void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 41
static void Msp430ClockP__Msp430ClockInit__default__initClocks(void );
# 62 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );
# 62 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Init.nc"
static error_t Msp430ClockP__Init__init(void );
# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(
# 51 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x10b5f3458);
# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 51 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x10b5f3458);
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
# 44 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t time);
# 42 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 44 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t time);
# 42 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 44 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t time);
# 42 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 44 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t time);
# 42 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );
#line 47
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );
# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );
# 41 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t delta);
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 44 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t time);
# 42 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 44 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t time);
# 42 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void );
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 44 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t time);
# 42 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 44 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t time);
# 42 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 44 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t time);
# 42 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 44 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t time);
# 42 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 76 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 67 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 56 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x10b4ee328);
# 75 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 56 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x10b4ee328);
# 57 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
#line 72
static void SchedulerBasicP__Scheduler__taskLoop(void );
#line 65
static bool SchedulerBasicP__Scheduler__runNextTask(void );
# 60 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Boot.nc"
static void TestTmp102C__Boot__booted(void );
# 83 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Timer.nc"
static void TestTmp102C__TestTimer__fired(void );
# 63 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Read.nc"
static void TestTmp102C__TempSensor__readDone(error_t result, TestTmp102C__TempSensor__val_t val);
# 62 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 100 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Leds.nc"
static void LedsP__Leds__led2Toggle(void );
# 99 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIORenP__33__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIORenP__33__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIORenP__34__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIORenP__34__IO__selectModuleFunc(void );
#line 85
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIORenP__36__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIORenP__36__IO__set(void );









static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__toggle(void );
#line 85
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__set(void );
#line 85
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIORenP__38__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIORenP__38__IO__set(void );
# 46 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );





static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );

static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void );



static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 103 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );
# 62 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 64 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
#line 64
static /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
# 109 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);
#line 116
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
# 82 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 75 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );
# 78 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
# 136 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
#line 129
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );
# 75 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
# 83 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );
#line 83
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x10bae15d8);
# 64 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x10bae15d8, 
# 64 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Timer.nc"
uint32_t dt);








static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x10bae15d8, 
# 73 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Timer.nc"
uint32_t dt);
# 82 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 53 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceRequested.nc"
static void SimpleTMP102P__ResourceRequested__requested(void );
# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Read.nc"
static error_t SimpleTMP102P__Read__read(void );
# 83 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Timer.nc"
static void SimpleTMP102P__TimerSensor__fired(void );
#line 83
static void SimpleTMP102P__TimerFail__fired(void );
# 112 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/I2CPacket.nc"
static void SimpleTMP102P__I2CBasicAddr__writeDone(error_t error, uint16_t addr, uint8_t length, 
#line 109
uint8_t * data);
#line 102
static void SimpleTMP102P__I2CBasicAddr__readDone(error_t error, uint16_t addr, uint8_t length, 
#line 99
uint8_t * data);
# 102 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
static void SimpleTMP102P__Resource__granted(void );
# 75 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/TaskBasic.nc"
static void SimpleTMP102P__calculateTemp__runTask(void );
# 65 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__ResourceConfigure__unconfigure(
# 53 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
uint8_t arg_0x10bc1b4b8);
# 59 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__ResourceConfigure__configure(
# 53 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
uint8_t arg_0x10bc1b4b8);
# 59 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Interrupts__rxDone(uint8_t data);
#line 54
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Interrupts__txDone(void );
# 76 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/I2CPacket.nc"
static error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__I2CBasicAddr__read(i2c_flags_t flags, uint16_t addr, uint8_t length, 
#line 72
uint8_t * data);
#line 92
static error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__I2CBasicAddr__write(i2c_flags_t flags, uint16_t addr, uint8_t length, 
#line 88
uint8_t * data);
# 120 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
static error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Resource__release(
# 52 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
uint8_t arg_0x10bc1d328);
# 88 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
static error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Resource__request(
# 52 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
uint8_t arg_0x10bc1d328);
# 102 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Resource__default__granted(
# 52 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
uint8_t arg_0x10bc1d328);
# 128 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
static bool /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Resource__isOwner(
# 52 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
uint8_t arg_0x10bc1d328);
# 120 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
static error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__default__release(
# 57 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
uint8_t arg_0x10bc15020);
# 88 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
static error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__default__request(
# 57 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
uint8_t arg_0x10bc15020);
# 102 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__granted(
# 57 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
uint8_t arg_0x10bc15020);
# 128 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
static bool /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__default__isOwner(
# 57 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
uint8_t arg_0x10bc15020);
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CConfigure.nc"
static msp430_i2c_union_config_t */*Msp430I2C1P.Z1UsciP*/Z1UsciP__0__Msp430I2CConfigure__getConfig(
# 42 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/platforms/z1/chips/msp430/usci/Z1UsciP.nc"
uint8_t arg_0x10bc874c0);
# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciRawInterrupts.nc"
static void HplMsp430UsciB1P__UsciRawInterrupts__rxDone(uint8_t data);
#line 53
static void HplMsp430UsciB1P__UsciRawInterrupts__txDone(void );
# 192 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
static void HplMsp430UsciB1P__Usci__setSlaveAddress(uint16_t addr);
#line 164
static void HplMsp430UsciB1P__Usci__setTransmitMode(void );
#line 184
static bool HplMsp430UsciB1P__Usci__getStopBit(void );
#line 93
static void HplMsp430UsciB1P__Usci__enableRxIntr(void );
#line 81
static void HplMsp430UsciB1P__Usci__resetUsci(bool reset);
#line 100
static void HplMsp430UsciB1P__Usci__clrRxIntr(void );
#line 191
static uint16_t HplMsp430UsciB1P__Usci__getSlaveAddress(void );
#line 92
static void HplMsp430UsciB1P__Usci__disableIntr(void );
#line 154
static void HplMsp430UsciB1P__Usci__enableI2C(void );
#line 73
static uint8_t HplMsp430UsciB1P__Usci__getUstat(void );
#line 186
static bool HplMsp430UsciB1P__Usci__getTransmitReceiveMode(void );
#line 94
static void HplMsp430UsciB1P__Usci__enableTxIntr(void );
#line 170
static void HplMsp430UsciB1P__Usci__setTXStart(void );
#line 101
static void HplMsp430UsciB1P__Usci__clrIntr(void );
#line 155
static void HplMsp430UsciB1P__Usci__disableI2C(void );
#line 169
static void HplMsp430UsciB1P__Usci__setTXStop(void );
#line 64
static void HplMsp430UsciB1P__Usci__setUbr(uint16_t ubr);
#line 161
static void HplMsp430UsciB1P__Usci__setModeI2C(msp430_i2c_union_config_t *config);
#line 108
static void HplMsp430UsciB1P__Usci__tx(uint8_t data);
#line 185
static bool HplMsp430UsciB1P__Usci__getStartBit(void );
#line 115
static uint8_t HplMsp430UsciB1P__Usci__rx(void );
#line 165
static void HplMsp430UsciB1P__Usci__setReceiveMode(void );
#line 99
static void HplMsp430UsciB1P__Usci__clrTxIntr(void );
# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciRawInterrupts.nc"
static void HplMsp430UsciAB1RawInterruptsP__UsciA__default__rxDone(uint8_t data);
#line 53
static void HplMsp430UsciAB1RawInterruptsP__UsciA__default__txDone(void );
# 59 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__rxDone(
# 41 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430UsciShareP.nc"
uint8_t arg_0x10bd75108, 
# 59 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
uint8_t data);
#line 54
static void /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__txDone(
# 41 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430UsciShareP.nc"
uint8_t arg_0x10bd75108);
# 59 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__rxDone(uint8_t data);
#line 54
static void /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__txDone(void );
# 62 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Init.nc"
static error_t /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );
# 79 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
#line 53
static bool /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );








static bool /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 53 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(
# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
uint8_t arg_0x10bda8020);
# 65 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(
# 60 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
uint8_t arg_0x10bda6340);
# 59 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(
# 60 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
uint8_t arg_0x10bda6340);
# 56 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 73
static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void );
#line 46
static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void );
# 120 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
static error_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(
# 54 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
uint8_t arg_0x10bdabd40);
# 88 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
static error_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(
# 54 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
uint8_t arg_0x10bdabd40);
# 102 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(
# 54 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
uint8_t arg_0x10bdabd40);
# 128 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
static bool /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(
# 54 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
uint8_t arg_0x10bdabd40);
# 90 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );
# 75 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
# 62 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Init.nc"
static error_t PlatformP__Msp430ClockInit__init(void );
#line 62
static error_t PlatformP__LedsInit__init(void );
# 50 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/platforms/z1/PlatformP.nc"
static inline error_t PlatformP__Init__init(void );
# 43 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__initTimerB(void );
#line 42
static void Msp430ClockP__Msp430ClockInit__initTimerA(void );
#line 41
static void Msp430ClockP__Msp430ClockInit__initClocks(void );
# 54 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/platforms/z1/chips/msp430/timer/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP__IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP__TACTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP__TAIV __asm ("0x012E");
static volatile uint16_t Msp430ClockP__TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP__TBIV __asm ("0x011E");

enum Msp430ClockP____nesc_unnamed4281 {

  Msp430ClockP__ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP__TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP__ACLK_CALIB_PERIOD
};

static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );
#line 82
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 117
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 132
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 152
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );





static inline void Msp430ClockP__startTimerA(void );
#line 180
static inline void Msp430ClockP__startTimerB(void );
#line 246
static inline error_t Msp430ClockP__Init__init(void );
# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(
# 51 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x10b5f3458);
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 126 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 51 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x10b5f3458);
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void );
# 62 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
#line 81
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
#line 126
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n);
# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time);
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void );
# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t;


static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time);
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void );
# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t;


static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time);
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void );
# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time);
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void );
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void );
# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void );
#line 85
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );
#line 130
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x);
#line 180
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time);
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void );
# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t;


static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time);
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void );
# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t;


static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time);
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void );
# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time);
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void );
# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time);
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void );
# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time);
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void );
# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP__VectorTimerB1__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerA0__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerA1__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerB0__fired(void );
# 11 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0032)))  ;
void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0030)))  ;
void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x003A)))  ;
void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0038)))  ;
# 62 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void );
# 59 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/platforms/z1/chips/msp430/McuSleepC.nc"
bool McuSleepC__dirty = TRUE;
mcu_power_t McuSleepC__powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC__msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC__getPowerState(void );
#line 107
static inline void McuSleepC__computePowerState(void );




static inline void McuSleepC__McuSleep__sleep(void );
# 62 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__init(void );
# 60 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Boot.nc"
static void RealMainP__Boot__booted(void );
# 62 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Init.nc"
static error_t RealMainP__PlatformInit__init(void );
# 57 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Scheduler.nc"
static void RealMainP__Scheduler__init(void );
#line 72
static void RealMainP__Scheduler__taskLoop(void );
#line 65
static bool RealMainP__Scheduler__runNextTask(void );
# 63 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/RealMainP.nc"
int main(void )   ;
# 75 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
# 56 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x10b4ee328);
# 76 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 61 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4282 {

  SchedulerBasicP__NUM_TASKS = 4U, 
  SchedulerBasicP__NO_TASK = 255
};

uint8_t SchedulerBasicP__m_head;
uint8_t SchedulerBasicP__m_tail;
uint8_t SchedulerBasicP__m_next[SchedulerBasicP__NUM_TASKS];








static __inline uint8_t SchedulerBasicP__popTask(void );
#line 97
static inline bool SchedulerBasicP__isWaiting(uint8_t id);




static inline bool SchedulerBasicP__pushTask(uint8_t id);
#line 124
static inline void SchedulerBasicP__Scheduler__init(void );









static bool SchedulerBasicP__Scheduler__runNextTask(void );
#line 149
static inline void SchedulerBasicP__Scheduler__taskLoop(void );
#line 170
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id);




static inline void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id);
# 64 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Timer.nc"
static void TestTmp102C__TestTimer__startPeriodic(uint32_t dt);
# 100 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Leds.nc"
static void TestTmp102C__Leds__led2Toggle(void );
# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Read.nc"
static error_t TestTmp102C__TempSensor__read(void );
# 56 "TestTmp102C.nc"
static inline void TestTmp102C__printTitles(void );









static inline void TestTmp102C__Boot__booted(void );





static inline void TestTmp102C__TestTimer__fired(void );



static void TestTmp102C__TempSensor__readDone(error_t error, uint16_t data);
# 46 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/GeneralIO.nc"
static void LedsP__Led0__makeOutput(void );
#line 40
static void LedsP__Led0__set(void );





static void LedsP__Led1__makeOutput(void );
#line 40
static void LedsP__Led1__set(void );

static void LedsP__Led2__toggle(void );



static void LedsP__Led2__makeOutput(void );
#line 40
static void LedsP__Led2__set(void );
# 56 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void );
#line 114
static inline void LedsP__Leds__led2Toggle(void );
# 57 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIORenP__33__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIORenP__33__IO__selectIOFunc(void );
#line 57
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIORenP__34__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIORenP__34__IO__selectIOFunc(void );
#line 48
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIORenP__36__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIORenP__36__IO__makeOutput(void );
#line 48
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__set(void );

static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__toggle(void );




static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__makeOutput(void );
#line 48
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIORenP__38__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIORenP__38__IO__makeOutput(void );
# 85 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void );
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );





static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
# 85 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void );
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );





static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle(void );
#line 85
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void );
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );

static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void );



static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
# 41 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void );
# 78 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void );
# 57 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void );
#line 47
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void );
#line 44
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 53 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
#line 65
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 114
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void );
# 82 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void );
# 49 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 64 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void );
# 67 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC__0____nesc_unnamed4283 {

  TransformCounterC__0__LOW_SHIFT_RIGHT = 5, 
  TransformCounterC__0__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT, 
  TransformCounterC__0__NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) + 5, 



  TransformCounterC__0__OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
#line 133
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
# 78 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void );
# 64 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void );
# 77 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0____nesc_unnamed4284 {

  TransformAlarmC__0__MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type ) - 1 - 5, 
  TransformAlarmC__0__MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void );
#line 147
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt);
#line 162
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
#line 177
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 67 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void );
# 109 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt);
#line 116
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void );
# 83 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void );
# 74 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4285 {
#line 74
  AlarmToTimerC__0__fired = 0U
};
#line 74
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired];
#line 55
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot);
#line 71
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );


static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
#line 93
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
# 67 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void );
# 136 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void );
#line 129
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x10bae15d8);
#line 71
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4286 {
#line 71
  VirtualizeTimerC__0__updateFromTimer = 1U
};
#line 71
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 53
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4287 {

  VirtualizeTimerC__0__NUM_TIMERS = 3U, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 59
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4288 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now);
#line 100
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
#line 139
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt);
#line 204
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num);
# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 63 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Read.nc"
static void SimpleTMP102P__Read__readDone(error_t result, SimpleTMP102P__Read__val_t val);
# 73 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Timer.nc"
static void SimpleTMP102P__TimerSensor__startOneShot(uint32_t dt);
# 76 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/I2CPacket.nc"
static error_t SimpleTMP102P__I2CBasicAddr__read(i2c_flags_t flags, uint16_t addr, uint8_t length, 
#line 72
uint8_t * data);
#line 92
static error_t SimpleTMP102P__I2CBasicAddr__write(i2c_flags_t flags, uint16_t addr, uint8_t length, 
#line 88
uint8_t * data);
# 120 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
static error_t SimpleTMP102P__Resource__release(void );
#line 88
static error_t SimpleTMP102P__Resource__request(void );
#line 128
static bool SimpleTMP102P__Resource__isOwner(void );
# 67 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/TaskBasic.nc"
static error_t SimpleTMP102P__calculateTemp__postTask(void );
# 66 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/tmp102/SimpleTMP102P.nc"
enum SimpleTMP102P____nesc_unnamed4289 {
#line 66
  SimpleTMP102P__calculateTemp = 2U
};
#line 66
typedef int SimpleTMP102P____nesc_sillytask_calculateTemp[SimpleTMP102P__calculateTemp];
#line 59
uint16_t SimpleTMP102P__temp;
uint8_t SimpleTMP102P__pointer;
uint8_t SimpleTMP102P__temperaturebuff[2];


uint8_t SimpleTMP102P__tempcmd;

static inline void SimpleTMP102P__calculateTemp__runTask(void );








static inline error_t SimpleTMP102P__Read__read(void );







static inline void SimpleTMP102P__TimerSensor__fired(void );



static inline void SimpleTMP102P__TimerFail__fired(void );



static inline void SimpleTMP102P__Resource__granted(void );










static inline void SimpleTMP102P__I2CBasicAddr__readDone(error_t error, uint16_t addr, uint8_t length, uint8_t *data);
#line 116
static void SimpleTMP102P__I2CBasicAddr__writeDone(error_t error, uint16_t addr, uint8_t length, uint8_t *data);










static inline void SimpleTMP102P__ResourceRequested__requested(void );
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CConfigure.nc"
static msp430_i2c_union_config_t */*Msp430I2C1P.I2CP*/Msp430I2CP__0__Msp430I2CConfigure__getConfig(
# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
uint8_t arg_0x10bc14220);
# 112 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/I2CPacket.nc"
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__I2CBasicAddr__writeDone(error_t error, uint16_t addr, uint8_t length, 
#line 109
uint8_t * data);
#line 102
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__I2CBasicAddr__readDone(error_t error, uint16_t addr, uint8_t length, 
#line 99
uint8_t * data);
# 192 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setSlaveAddress(uint16_t addr);
#line 164
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setTransmitMode(void );
#line 184
static bool /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getStopBit(void );
#line 93
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__enableRxIntr(void );






static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__clrRxIntr(void );
#line 191
static uint16_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getSlaveAddress(void );
#line 92
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__disableIntr(void );
#line 73
static uint8_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getUstat(void );
#line 186
static bool /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getTransmitReceiveMode(void );
#line 94
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__enableTxIntr(void );
#line 170
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setTXStart(void );
#line 101
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__clrIntr(void );
#line 155
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__disableI2C(void );
#line 169
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setTXStop(void );
#line 161
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setModeI2C(msp430_i2c_union_config_t *config);
#line 108
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__tx(uint8_t data);
#line 185
static bool /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getStartBit(void );
#line 115
static uint8_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__rx(void );
#line 165
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setReceiveMode(void );
#line 99
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__clrTxIntr(void );
# 102 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Resource__granted(
# 52 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
uint8_t arg_0x10bc1d328);
# 120 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
static error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__release(
# 57 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
uint8_t arg_0x10bc15020);
# 88 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
static error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__request(
# 57 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
uint8_t arg_0x10bc15020);
# 128 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
static bool /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__isOwner(
# 57 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
uint8_t arg_0x10bc15020);







enum /*Msp430I2C1P.I2CP*/Msp430I2CP__0____nesc_unnamed4290 {




  Msp430I2CP__0__TIMEOUT = 1200
};

uint8_t */*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_buf;
uint8_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_len;
uint8_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_pos;
i2c_flags_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_flags;

static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__nextRead(void );
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__nextWrite(void );
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__signalDone(error_t error);





static inline error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Resource__request(uint8_t id);



static inline uint8_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Resource__isOwner(uint8_t id);



static inline error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Resource__release(uint8_t id);



static inline void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__ResourceConfigure__configure(uint8_t id);



static inline void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__ResourceConfigure__unconfigure(uint8_t id);



static inline void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__granted(uint8_t id);



static inline error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__default__request(uint8_t id);

static inline error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__default__release(uint8_t id);
static inline void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Resource__default__granted(uint8_t id);





static inline error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__I2CBasicAddr__read(i2c_flags_t flags, 
uint16_t addr, uint8_t len, 
uint8_t *buf);
#line 158
static inline error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__I2CBasicAddr__write(i2c_flags_t flags, 
uint16_t addr, uint8_t len, 
uint8_t *buf);
#line 199
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__nextRead(void );
#line 228
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__nextWrite(void );
#line 254
static inline void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Interrupts__txDone(void );







static inline void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Interrupts__rxDone(uint8_t data);







static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__signalDone(error_t error);







static inline error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__default__isOwner(uint8_t id);
# 61 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/platforms/z1/chips/msp430/usci/Z1UsciP.nc"
msp430_i2c_union_config_t /*Msp430I2C1P.Z1UsciP*/Z1UsciP__0__msp430_i2c_z1_config = { { 
.ucmode = 3, 
.ucmst = 1, 
.ucmm = 0, 
.ucsla10 = 0, 
.uca10 = 0, 
.uctr = 0, 
.ucssel = 2, 
.i2coa = 1, 
.ucgcen = 1, 
.ubr = 80 } };


static inline msp430_i2c_union_config_t */*Msp430I2C1P.Z1UsciP*/Z1UsciP__0__Msp430I2CConfigure__getConfig(uint8_t id);
# 59 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
static void HplMsp430UsciB1P__Interrupts__rxDone(uint8_t data);
#line 54
static void HplMsp430UsciB1P__Interrupts__txDone(void );
# 99 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430UsciB1P__USCL__selectIOFunc(void );
#line 92
static void HplMsp430UsciB1P__USCL__selectModuleFunc(void );






static void HplMsp430UsciB1P__USDA__selectIOFunc(void );
#line 92
static void HplMsp430UsciB1P__USDA__selectModuleFunc(void );
# 75 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static volatile uint8_t HplMsp430UsciB1P__UC1IE __asm ("0x0006");
static volatile uint8_t HplMsp430UsciB1P__UC1IFG __asm ("0x0007");
static volatile uint8_t HplMsp430UsciB1P__UCB1CTL0 __asm ("0x00D8");
static volatile uint8_t HplMsp430UsciB1P__UCB1CTL1 __asm ("0x00D9");
static volatile uint8_t HplMsp430UsciB1P__UCB1RXBUF __asm ("0x00DE");
static volatile uint8_t HplMsp430UsciB1P__UCB1TXBUF __asm ("0x00DF");
static volatile uint16_t HplMsp430UsciB1P__UCB1I2COA __asm ("0x017C");
static volatile uint8_t HplMsp430UsciB1P__UCB1I2CIE __asm ("0x00DC");

static inline void HplMsp430UsciB1P__UsciRawInterrupts__rxDone(uint8_t temp);



static inline void HplMsp430UsciB1P__UsciRawInterrupts__txDone(void );
#line 109
static inline void HplMsp430UsciB1P__Usci__setUbr(uint16_t control);
#line 124
static inline uint8_t HplMsp430UsciB1P__Usci__getUstat(void );




static inline void HplMsp430UsciB1P__Usci__resetUsci(bool reset);
#line 207
static inline void HplMsp430UsciB1P__Usci__clrTxIntr(void );



static inline void HplMsp430UsciB1P__Usci__clrRxIntr(void );



static inline void HplMsp430UsciB1P__Usci__clrIntr(void );
#line 227
static inline void HplMsp430UsciB1P__Usci__disableIntr(void );



static inline void HplMsp430UsciB1P__Usci__enableRxIntr(void );






static inline void HplMsp430UsciB1P__Usci__enableTxIntr(void );
#line 252
static inline void HplMsp430UsciB1P__Usci__tx(uint8_t data);



static inline uint8_t HplMsp430UsciB1P__Usci__rx(void );










static inline void HplMsp430UsciB1P__Usci__enableI2C(void );
#line 281
static inline void HplMsp430UsciB1P__Usci__disableI2C(void );






static inline void HplMsp430UsciB1P__configI2C(msp430_i2c_union_config_t *config);








static inline void HplMsp430UsciB1P__Usci__setModeI2C(msp430_i2c_union_config_t *config);
#line 331
static inline void HplMsp430UsciB1P__Usci__setTransmitMode(void );
static inline void HplMsp430UsciB1P__Usci__setReceiveMode(void );



static inline void HplMsp430UsciB1P__Usci__setTXStop(void );
static inline void HplMsp430UsciB1P__Usci__setTXStart(void );










static inline bool HplMsp430UsciB1P__Usci__getStartBit(void );
static inline bool HplMsp430UsciB1P__Usci__getStopBit(void );
static inline bool HplMsp430UsciB1P__Usci__getTransmitReceiveMode(void );


static uint16_t HplMsp430UsciB1P__Usci__getSlaveAddress(void );
static inline void HplMsp430UsciB1P__Usci__setSlaveAddress(uint16_t addr);
# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciRawInterrupts.nc"
static void HplMsp430UsciAB1RawInterruptsP__UsciA__rxDone(uint8_t data);
#line 53
static void HplMsp430UsciAB1RawInterruptsP__UsciA__txDone(void );




static void HplMsp430UsciAB1RawInterruptsP__UsciB__rxDone(uint8_t data);
#line 53
static void HplMsp430UsciAB1RawInterruptsP__UsciB__txDone(void );
# 54 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciAB1RawInterruptsP.nc"
void sig_USCIAB1RX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0022)))  ;
#line 73
void sig_USCIAB1TX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0020)))  ;
#line 95
static inline void HplMsp430UsciAB1RawInterruptsP__UsciA__default__txDone(void );



static inline void HplMsp430UsciAB1RawInterruptsP__UsciA__default__rxDone(uint8_t temp);
# 59 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__Interrupts__rxDone(
# 41 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430UsciShareP.nc"
uint8_t arg_0x10bd75108, 
# 59 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
uint8_t data);
#line 54
static void /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__Interrupts__txDone(
# 41 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430UsciShareP.nc"
uint8_t arg_0x10bd75108);
# 90 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__userId(void );
# 49 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__txDone(void );




static void /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__rxDone(uint8_t data);




static inline void /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data);
# 49 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0____nesc_unnamed4291 {
#line 49
  FcfsResourceQueueC__0__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[2U];
uint8_t /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead = /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
uint8_t /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail = /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

static inline error_t /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );




static inline bool /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );



static inline bool /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
#line 82
static inline error_t /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
# 53 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(
# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
uint8_t arg_0x10bda8020);
# 65 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(
# 60 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
uint8_t arg_0x10bda6340);
# 59 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(
# 60 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
uint8_t arg_0x10bda6340);
# 79 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(resource_client_id_t id);
#line 53
static bool /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void );
#line 70
static resource_client_id_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void );
# 73 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested(void );
#line 46
static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void );
# 102 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(
# 54 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
uint8_t arg_0x10bdabd40);
# 67 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void );
# 75 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
enum /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4292 {
#line 75
  ArbiterP__0__grantedTask = 3U
};
#line 75
typedef int /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_sillytask_grantedTask[/*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask];
#line 67
enum /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4293 {
#line 67
  ArbiterP__0__RES_CONTROLLED, ArbiterP__0__RES_GRANTING, ArbiterP__0__RES_IMM_GRANTING, ArbiterP__0__RES_BUSY
};
#line 68
enum /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4294 {
#line 68
  ArbiterP__0__default_owner_id = 2U
};
#line 69
enum /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4295 {
#line 69
  ArbiterP__0__NO_RES = 0xFF
};
uint8_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
uint8_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
uint8_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;



static inline error_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(uint8_t id);
#line 111
static error_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id);
#line 133
static inline error_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 153
static bool /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );
#line 166
static uint8_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );










static bool /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id);
#line 190
static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
#line 202
static inline void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id);

static inline void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(uint8_t id);



static inline void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void );

static inline void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void );





static inline void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id);
# 408 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
  __eint();
}

# 196 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void )
{
}

# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void ){
#line 48
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow();
#line 48
}
#line 48
# 137 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n)
{
}

# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x10b5f3458){
#line 39
  switch (arg_0x10b5f3458) {
#line 39
    case 0:
#line 39
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired();
#line 39
      break;
#line 39
    case 1:
#line 39
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired();
#line 39
      break;
#line 39
    case 2:
#line 39
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired();
#line 39
      break;
#line 39
    case 5:
#line 39
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired();
#line 39
      break;
#line 39
    default:
#line 39
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x10b5f3458);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 126 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(0);
}

# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA0__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired();
#line 39
}
#line 39
# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4296 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(* (volatile uint16_t * )354U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n)
{
}

# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void )
{
}

# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired();
#line 45
}
#line 45
# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4297 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(* (volatile uint16_t * )356U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n)
{
}

# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void )
{
}

# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired();
#line 45
}
#line 45
# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4298 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(* (volatile uint16_t * )358U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n)
{
}

# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void )
{
}

# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired();
#line 45
}
#line 45
# 131 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 134
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(n >> 1);
}

# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA1__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired();
#line 39
}
#line 39
# 126 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(0);
}

# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB0__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired();
#line 39
}
#line 39
# 196 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void )
{
}

# 114 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void )
{
}

# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void )
{
}

# 177 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void )
{
}

# 82 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void ){
#line 82
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow();
#line 82
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow();
#line 82
}
#line 82
# 133 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC__0__m_upper & /*CounterMilli32C.Transform*/TransformCounterC__0__OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow();
      }
  }
}

# 82 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
#line 82
  /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow();
#line 82
}
#line 82
# 64 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow();
}

# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 48
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow();
#line 48
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow();
#line 48
}
#line 48
# 137 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow();
}

# 67 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 81 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void )
{
#line 82
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask();
}

# 78 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired();
#line 78
}
#line 78
# 162 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
      }
  }
}

# 78 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired();
#line 78
}
#line 78
# 135 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents();
#line 58
}
#line 58
# 70 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired();
}

# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 45
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired();
#line 45
}
#line 45
# 150 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n)
{
}

# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4299 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(* (volatile uint16_t * )386U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired();
    }
}

# 97 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP__isWaiting(uint8_t id)
{
  return SchedulerBasicP__m_next[id] != SchedulerBasicP__NO_TASK || SchedulerBasicP__m_tail == id;
}

static inline bool SchedulerBasicP__pushTask(uint8_t id)
{
  if (!SchedulerBasicP__isWaiting(id)) 
    {
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_head = id;
          SchedulerBasicP__m_tail = id;
        }
      else 
        {
          SchedulerBasicP__m_next[SchedulerBasicP__m_tail] = id;
          SchedulerBasicP__m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 49 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get();
}

# 64 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 81 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void )
{
  return * (volatile uint16_t * )384U & 1U;
}

# 46 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void ){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending();
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 54 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending();
}

# 71 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Counter.nc"
inline static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 130 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void )
{
  * (volatile uint16_t * )386U |= 0x0010;
}

# 57 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents();
#line 57
}
#line 57
# 95 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )386U &= ~0x0001;
}

# 44 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 155 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )402U = x;
}

# 41 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(time);
#line 41
}
#line 41
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 165 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get() + x;
}

# 43 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 81 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 87
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 94
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 97
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 99
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents();
  }
}

# 103 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 192 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void )
{
}

# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired();
#line 45
}
#line 45
# 150 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n)
{
}

# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4300 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(* (volatile uint16_t * )388U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void )
{
}

# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired();
#line 45
}
#line 45
# 150 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n)
{
}

# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4301 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(* (volatile uint16_t * )390U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void )
{
}

# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired();
#line 45
}
#line 45
# 150 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n)
{
}

# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4302 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(* (volatile uint16_t * )392U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void )
{
}

# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired();
#line 45
}
#line 45
# 150 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n)
{
}

# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4303 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(* (volatile uint16_t * )394U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void )
{
}

# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired();
#line 45
}
#line 45
# 150 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n)
{
}

# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4304 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(* (volatile uint16_t * )396U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void )
{
}

# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired();
#line 45
}
#line 45
# 150 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n)
{
}

# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4305 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(* (volatile uint16_t * )398U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired();
    }
}

# 131 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 134
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(n >> 1);
}

# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB1__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired();
#line 39
}
#line 39
# 124 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__Scheduler__init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

# 57 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__init(void ){
#line 57
  SchedulerBasicP__Scheduler__init();
#line 57
}
#line 57
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__set(void )
#line 48
{
  /* atomic removed: atomic calls only */
#line 48
  * (volatile uint8_t * )49U |= 0x01 << 5;
}

# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__set();
#line 48
}
#line 48
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set();
}

# 40 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__set(void ){
#line 40
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set();
#line 40
}
#line 40
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIORenP__38__IO__set(void )
#line 48
{
  /* atomic removed: atomic calls only */
#line 48
  * (volatile uint8_t * )49U |= 0x01 << 6;
}

# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIORenP__38__IO__set();
#line 48
}
#line 48
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set();
}

# 40 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__set(void ){
#line 40
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set();
#line 40
}
#line 40
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIORenP__36__IO__set(void )
#line 48
{
  /* atomic removed: atomic calls only */
#line 48
  * (volatile uint8_t * )49U |= 0x01 << 4;
}

# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIORenP__36__IO__set();
#line 48
}
#line 48
# 48 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set();
}

# 40 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__set(void ){
#line 40
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set();
#line 40
}
#line 40
# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__makeOutput(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 55
  * (volatile uint8_t * )50U |= 0x01 << 5;
}

# 85 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__makeOutput();
#line 85
}
#line 85
# 54 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput();
}

# 46 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput();
#line 46
}
#line 46
# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIORenP__38__IO__makeOutput(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 55
  * (volatile uint8_t * )50U |= 0x01 << 6;
}

# 85 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIORenP__38__IO__makeOutput();
#line 85
}
#line 85
# 54 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput();
}

# 46 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput();
#line 46
}
#line 46
# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIORenP__36__IO__makeOutput(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 55
  * (volatile uint8_t * )50U |= 0x01 << 4;
}

# 85 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIORenP__36__IO__makeOutput();
#line 85
}
#line 85
# 54 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput();
}

# 46 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput();
#line 46
}
#line 46
# 56 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 57
  {
    ;
    LedsP__Led0__makeOutput();
    LedsP__Led1__makeOutput();
    LedsP__Led2__makeOutput();
    LedsP__Led0__set();
    LedsP__Led1__set();
    LedsP__Led2__set();
  }
  return SUCCESS;
}

# 62 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Init.nc"
inline static error_t PlatformP__LedsInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = LedsP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 108 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/platforms/z1/chips/msp430/timer/Msp430XDcoCalib.h"
static inline void Set_DCO(unsigned int Delta)
{
  unsigned int Compare;
#line 110
  unsigned int Oldcapture = 0;

  BCSCTL1 |= 0x30;
  TACCTL2 = 0x4000 + 0x1000 + 0x0100;
  TACTL = 0x0200 + 0x0020 + 0x0004;

  while (1) 
    {
      while (!(0x0001 & TACCTL2)) ;
      TACCTL2 &= ~0x0001;
      Compare = TACCR2;
      Compare = Compare - Oldcapture;
      Oldcapture = TACCR2;

      if (Delta == Compare) {
        break;
        }
      else {
#line 126
        if (Delta < Compare) 
          {
            DCOCTL--;
            if (DCOCTL == 0xFF) {
              if (BCSCTL1 & 0x0f) {
                BCSCTL1--;
                }
              }
          }
        else 
#line 134
          {
            DCOCTL++;
            if (DCOCTL == 0x00) {
              if ((BCSCTL1 & 0x0f) != 0x0f) {
                BCSCTL1++;
                }
              }
          }
        }
    }
#line 141
  TACCTL2 = 0;
  TACTL = 0;
  BCSCTL1 &= ~0x30;
}

# 180 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/platforms/z1/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__startTimerB(void )
{

  Msp430ClockP__TBCTL = 0x0020 | (Msp430ClockP__TBCTL & ~(0x0020 | 0x0010));
}

#line 168
static inline void Msp430ClockP__startTimerA(void )
{

  Msp430ClockP__TACTL = 0x0020 | (Msp430ClockP__TACTL & ~(0x0020 | 0x0010));
}

#line 132
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void )
{
  TBR = 0;









  Msp430ClockP__TBCTL = 0x0100 | 0x0002;
}

#line 162
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerB();
}

# 43 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerB(void ){
#line 43
  Msp430ClockP__Msp430ClockInit__default__initTimerB();
#line 43
}
#line 43
# 117 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/platforms/z1/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void )
{
  TAR = 0;









  Msp430ClockP__TACTL = 0x0200 | 0x0002;
}

#line 157
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerA();
}

# 42 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerA(void ){
#line 42
  Msp430ClockP__Msp430ClockInit__default__initTimerA();
#line 42
}
#line 42
# 82 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/platforms/z1/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void )
{


  if (CALBC1_8MHZ != 0xFF) {
      DCOCTL = 0x00;
      BCSCTL1 = CALBC1_8MHZ;
      DCOCTL = CALDCO_8MHZ;
    }
  else 
#line 90
    {
      DCOCTL = 0x00;
      BCSCTL1 = 0x8D;
      DCOCTL = 0x88;
    }







  BCSCTL1 = 0x80 | BCSCTL1;
#line 114
  Msp430ClockP__IE1 &= ~0x02;
}

#line 152
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitClocks();
}

# 41 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initClocks(void ){
#line 41
  Msp430ClockP__Msp430ClockInit__default__initClocks();
#line 41
}
#line 41
# 246 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/platforms/z1/chips/msp430/timer/Msp430ClockP.nc"
static inline error_t Msp430ClockP__Init__init(void )
{

  Msp430ClockP__TACTL = 0x0004;
  Msp430ClockP__TAIV = 0;
  Msp430ClockP__TBCTL = 0x0004;
  Msp430ClockP__TBIV = 0;
  /* atomic removed: atomic calls only */

  {



    Msp430ClockP__Msp430ClockInit__initClocks();
    Msp430ClockP__Msp430ClockInit__initTimerA();
    Msp430ClockP__Msp430ClockInit__initTimerB();
    Msp430ClockP__startTimerA();
    Msp430ClockP__startTimerB();
  }
  Set_DCO(1953);
  return SUCCESS;
}

# 62 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Init.nc"
inline static error_t PlatformP__Msp430ClockInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Msp430ClockP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 50 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/platforms/z1/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
#line 50
{
  WDTCTL = 0x5A00 + 0x0080;
  PlatformP__Msp430ClockInit__init();
  PlatformP__LedsInit__init();
  return SUCCESS;
}

# 62 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Init.nc"
inline static error_t RealMainP__PlatformInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = PlatformP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 65 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Scheduler.nc"
inline static bool RealMainP__Scheduler__runNextTask(void ){
#line 65
  unsigned char __nesc_result;
#line 65

#line 65
  __nesc_result = SchedulerBasicP__Scheduler__runNextTask();
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 74 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/platforms/z1/chips/msp430/usci/Z1UsciP.nc"
static inline msp430_i2c_union_config_t */*Msp430I2C1P.Z1UsciP*/Z1UsciP__0__Msp430I2CConfigure__getConfig(uint8_t id)
#line 74
{
  return (msp430_i2c_union_config_t *)&/*Msp430I2C1P.Z1UsciP*/Z1UsciP__0__msp430_i2c_z1_config;
}

# 45 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CConfigure.nc"
inline static msp430_i2c_union_config_t */*Msp430I2C1P.I2CP*/Msp430I2CP__0__Msp430I2CConfigure__getConfig(uint8_t arg_0x10bc14220){
#line 45
  union __nesc_unnamed4277 *__nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430I2C1P.Z1UsciP*/Z1UsciP__0__Msp430I2CConfigure__getConfig(arg_0x10bc14220);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 129 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline void HplMsp430UsciB1P__Usci__resetUsci(bool reset)
#line 129
{
  if (reset) {
    HplMsp430UsciB1P__UCB1CTL1 |= 0x01;
    }
  else {
#line 133
    HplMsp430UsciB1P__UCB1CTL1 &= ~0x01;
    }
}

#line 109
static inline void HplMsp430UsciB1P__Usci__setUbr(uint16_t control)
#line 109
{
  /* atomic removed: atomic calls only */
#line 110
  {
    UCB1BR0 = control & 0x00FF;
    UCB1BR1 = (control >> 8) & 0x00FF;
  }
}

#line 288
static inline void HplMsp430UsciB1P__configI2C(msp430_i2c_union_config_t *config)
#line 288
{
  HplMsp430UsciB1P__UCB1CTL1 = config->i2cRegisters.uctl1 | 0x01;
  HplMsp430UsciB1P__UCB1CTL0 = config->i2cRegisters.uctl0 | 0x01;
  HplMsp430UsciB1P__Usci__setUbr(config->i2cRegisters.ubr);
  HplMsp430UsciB1P__UCB1I2COA = config->i2cRegisters.ui2coa;
  UCB1I2CSA = 0;
  HplMsp430UsciB1P__UCB1I2CIE = 0;
}

# 57 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIORenP__34__IO__selectModuleFunc(void )
#line 57
{
  /* atomic removed: atomic calls only */
#line 57
  * (volatile uint8_t * )51U |= 0x01 << 2;
}

# 92 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciB1P__USCL__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIORenP__34__IO__selectModuleFunc();
#line 92
}
#line 92
# 57 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIORenP__33__IO__selectModuleFunc(void )
#line 57
{
  /* atomic removed: atomic calls only */
#line 57
  * (volatile uint8_t * )51U |= 0x01 << 1;
}

# 92 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciB1P__USDA__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIORenP__33__IO__selectModuleFunc();
#line 92
}
#line 92
# 267 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline void HplMsp430UsciB1P__Usci__enableI2C(void )
#line 267
{
  /* atomic removed: atomic calls only */
#line 268
  {


    P5OUT &= ~0x06;
    P5REN &= ~0x06;
    P5OUT |= 0x06;
    P5REN |= 0x06;

    HplMsp430UsciB1P__USDA__selectModuleFunc();
    HplMsp430UsciB1P__USCL__selectModuleFunc();
  }
}

#line 215
static inline void HplMsp430UsciB1P__Usci__clrIntr(void )
#line 215
{
  HplMsp430UsciB1P__UC1IFG &= ~(0x08 | 0x04);
}









static inline void HplMsp430UsciB1P__Usci__disableIntr(void )
#line 227
{
  HplMsp430UsciB1P__UC1IE &= ~(0x08 | 0x04);
}

#line 297
static inline void HplMsp430UsciB1P__Usci__setModeI2C(msp430_i2c_union_config_t *config)
#line 297
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 298
    {
      HplMsp430UsciB1P__Usci__disableIntr();
      HplMsp430UsciB1P__Usci__clrIntr();
      HplMsp430UsciB1P__Usci__resetUsci(TRUE);
      HplMsp430UsciB1P__Usci__enableI2C();
      HplMsp430UsciB1P__configI2C(config);
      HplMsp430UsciB1P__Usci__resetUsci(FALSE);
    }
#line 305
    __nesc_atomic_end(__nesc_atomic); }
}

# 161 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setModeI2C(msp430_i2c_union_config_t *config){
#line 161
  HplMsp430UsciB1P__Usci__setModeI2C(config);
#line 161
}
#line 161
# 98 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
static inline void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__ResourceConfigure__configure(uint8_t id)
#line 98
{
  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setModeI2C(/*Msp430I2C1P.I2CP*/Msp430I2CP__0__Msp430I2CConfigure__getConfig(id));
}

# 216 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 216
{
}

# 59 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x10bda6340){
#line 59
  switch (arg_0x10bda6340) {
#line 59
    case /*TestTmp102AppC.Temperature.I2C.UsciC*/Msp430UsciB1C__0__CLIENT_ID:
#line 59
      /*Msp430I2C1P.I2CP*/Msp430I2CP__0__ResourceConfigure__configure(/*TestTmp102AppC.Temperature.I2C*/Msp430I2C1C__0__CLIENT_ID);
#line 59
      break;
#line 59
    default:
#line 59
      /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(arg_0x10bda6340);
#line 59
      break;
#line 59
    }
#line 59
}
#line 59
# 63 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Read.nc"
inline static void SimpleTMP102P__Read__readDone(error_t result, SimpleTMP102P__Read__val_t val){
#line 63
  TestTmp102C__TempSensor__readDone(result, val);
#line 63
}
#line 63
# 112 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
static inline error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__default__release(uint8_t id)
#line 112
{
#line 112
  return FAIL;
}

# 120 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
inline static error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__release(uint8_t arg_0x10bc15020){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  switch (arg_0x10bc15020) {
#line 120
    case /*TestTmp102AppC.Temperature.I2C*/Msp430I2C1C__0__CLIENT_ID:
#line 120
      __nesc_result = /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(/*TestTmp102AppC.Temperature.I2C.UsciC*/Msp430UsciB1C__0__CLIENT_ID);
#line 120
      break;
#line 120
    default:
#line 120
      __nesc_result = /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__default__release(arg_0x10bc15020);
#line 120
      break;
#line 120
    }
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 94 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
static inline error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Resource__release(uint8_t id)
#line 94
{
  return /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__release(id);
}

# 120 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
inline static error_t SimpleTMP102P__Resource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Resource__release(/*TestTmp102AppC.Temperature.I2C*/Msp430I2C1C__0__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 337 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline void HplMsp430UsciB1P__Usci__setTXStart(void )
#line 337
{
#line 337
  HplMsp430UsciB1P__UCB1CTL1 |= 0x02;
}

# 170 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setTXStart(void ){
#line 170
  HplMsp430UsciB1P__Usci__setTXStart();
#line 170
}
#line 170
# 124 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline uint8_t HplMsp430UsciB1P__Usci__getUstat(void )
#line 124
{
  return UCB1STAT;
}

# 73 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static uint8_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getUstat(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = HplMsp430UsciB1P__Usci__getUstat();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 349 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline bool HplMsp430UsciB1P__Usci__getStopBit(void )
#line 349
{
#line 349
  return HplMsp430UsciB1P__UCB1CTL1 & 0x04;
}

# 184 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static bool /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getStopBit(void ){
#line 184
  unsigned char __nesc_result;
#line 184

#line 184
  __nesc_result = HplMsp430UsciB1P__Usci__getStopBit();
#line 184

#line 184
  return __nesc_result;
#line 184
}
#line 184
# 238 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline void HplMsp430UsciB1P__Usci__enableTxIntr(void )
#line 238
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 239
    {
      HplMsp430UsciB1P__UC1IFG &= ~0x08;
      HplMsp430UsciB1P__UC1IE |= 0x08;
    }
#line 242
    __nesc_atomic_end(__nesc_atomic); }
}

# 94 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__enableTxIntr(void ){
#line 94
  HplMsp430UsciB1P__Usci__enableTxIntr();
#line 94
}
#line 94
# 354 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline void HplMsp430UsciB1P__Usci__setSlaveAddress(uint16_t addr)
#line 354
{
#line 354
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 354
    {
#line 354
      UCB1I2CSA = addr;
    }
#line 355
    __nesc_atomic_end(__nesc_atomic); }
}

# 192 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setSlaveAddress(uint16_t addr){
#line 192
  HplMsp430UsciB1P__Usci__setSlaveAddress(addr);
#line 192
}
#line 192
# 331 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline void HplMsp430UsciB1P__Usci__setTransmitMode(void )
#line 331
{
#line 331
  HplMsp430UsciB1P__UCB1CTL1 |= 0x10;
}

# 164 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setTransmitMode(void ){
#line 164
  HplMsp430UsciB1P__Usci__setTransmitMode();
#line 164
}
#line 164
# 158 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
static inline error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__I2CBasicAddr__write(i2c_flags_t flags, 
uint16_t addr, uint8_t len, 
uint8_t *buf)
#line 160
{
  uint16_t i = 0;

#line 162
  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_buf = buf;
  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_len = len;
  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_flags = flags;
  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_pos = 0;
  while (/*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getUstat() & 0x10) {
      if (i >= /*Msp430I2C1P.I2CP*/Msp430I2CP__0__TIMEOUT) {
          return FAIL;
        }
      i++;
    }

  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setTransmitMode();
  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setSlaveAddress(addr);
  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__enableTxIntr();

  if (flags & I2C_START) {
      while (/*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getStopBit()) {
          if (i >= /*Msp430I2C1P.I2CP*/Msp430I2CP__0__TIMEOUT) {
              return EBUSY;
            }
          i++;
        }
      i = 0;

      while (/*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getUstat() & 0x10) {
          if (i >= /*Msp430I2C1P.I2CP*/Msp430I2CP__0__TIMEOUT) {
              return FAIL;
            }
          i++;
        }
      /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setTXStart();
    }
  else 
#line 193
    {
      /*Msp430I2C1P.I2CP*/Msp430I2CP__0__nextWrite();
    }
  return SUCCESS;
}

# 92 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/I2CPacket.nc"
inline static error_t SimpleTMP102P__I2CBasicAddr__write(i2c_flags_t flags, uint16_t addr, uint8_t length, uint8_t * data){
#line 92
  unsigned char __nesc_result;
#line 92

#line 92
  __nesc_result = /*Msp430I2C1P.I2CP*/Msp430I2CP__0__I2CBasicAddr__write(flags, addr, length, data);
#line 92

#line 92
  return __nesc_result;
#line 92
}
#line 92
# 91 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/tmp102/SimpleTMP102P.nc"
static inline void SimpleTMP102P__Resource__granted(void )
#line 91
{
  error_t error;

#line 93
  SimpleTMP102P__pointer = 0x00;
  SimpleTMP102P__tempcmd = 1;
  error = SimpleTMP102P__I2CBasicAddr__write(I2C_START | I2C_STOP, 0x48, 1, &SimpleTMP102P__pointer);
  if (error) {
      SimpleTMP102P__Resource__release();
      SimpleTMP102P__Read__readDone(error, 0);
    }
}

# 113 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
static inline void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Resource__default__granted(uint8_t id)
#line 113
{
}

# 102 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
inline static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Resource__granted(uint8_t arg_0x10bc1d328){
#line 102
  switch (arg_0x10bc1d328) {
#line 102
    case /*TestTmp102AppC.Temperature.I2C*/Msp430I2C1C__0__CLIENT_ID:
#line 102
      SimpleTMP102P__Resource__granted();
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Resource__default__granted(arg_0x10bc1d328);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 106 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
static inline void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__granted(uint8_t id)
#line 106
{
  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Resource__granted(id);
}

# 202 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id)
#line 202
{
}

# 102 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
inline static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(uint8_t arg_0x10bdabd40){
#line 102
  switch (arg_0x10bdabd40) {
#line 102
    case /*TestTmp102AppC.Temperature.I2C.UsciC*/Msp430UsciB1C__0__CLIENT_ID:
#line 102
      /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__granted(/*TestTmp102AppC.Temperature.I2C*/Msp430I2C1C__0__CLIENT_ID);
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(arg_0x10bdabd40);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 101 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__clrIntr(void ){
#line 101
  HplMsp430UsciB1P__Usci__clrIntr();
#line 101
}
#line 101
#line 92
inline static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__disableIntr(void ){
#line 92
  HplMsp430UsciB1P__Usci__disableIntr();
#line 92
}
#line 92
# 112 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/I2CPacket.nc"
inline static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__I2CBasicAddr__writeDone(error_t error, uint16_t addr, uint8_t length, uint8_t * data){
#line 112
  SimpleTMP102P__I2CBasicAddr__writeDone(error, addr, length, data);
#line 112
}
#line 112
# 336 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline void HplMsp430UsciB1P__Usci__setTXStop(void )
#line 336
{
#line 336
  HplMsp430UsciB1P__UCB1CTL1 |= 0x04;
}

# 169 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setTXStop(void ){
#line 169
  HplMsp430UsciB1P__Usci__setTXStop();
#line 169
}
#line 169
# 348 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline bool HplMsp430UsciB1P__Usci__getStartBit(void )
#line 348
{
#line 348
  return HplMsp430UsciB1P__UCB1CTL1 & 0x02;
}

# 185 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static bool /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getStartBit(void ){
#line 185
  unsigned char __nesc_result;
#line 185

#line 185
  __nesc_result = HplMsp430UsciB1P__Usci__getStartBit();
#line 185

#line 185
  return __nesc_result;
#line 185
}
#line 185
# 231 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline void HplMsp430UsciB1P__Usci__enableRxIntr(void )
#line 231
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 232
    {
      HplMsp430UsciB1P__UC1IFG &= ~0x04;
      HplMsp430UsciB1P__UC1IE |= 0x04;
    }
#line 235
    __nesc_atomic_end(__nesc_atomic); }
}

# 93 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__enableRxIntr(void ){
#line 93
  HplMsp430UsciB1P__Usci__enableRxIntr();
#line 93
}
#line 93
# 332 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline void HplMsp430UsciB1P__Usci__setReceiveMode(void )
#line 332
{
#line 332
  HplMsp430UsciB1P__UCB1CTL1 &= ~0x10;
}

# 165 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setReceiveMode(void ){
#line 165
  HplMsp430UsciB1P__Usci__setReceiveMode();
#line 165
}
#line 165
# 119 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
static inline error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__I2CBasicAddr__read(i2c_flags_t flags, 
uint16_t addr, uint8_t len, 
uint8_t *buf)
#line 121
{
  uint16_t i = 0;

#line 123
  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_buf = buf;
  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_len = len;
  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_flags = flags;
  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_pos = 0;

  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setReceiveMode();
  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setSlaveAddress(addr);
  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__enableRxIntr();

  if (flags & I2C_START) {
      while (/*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getStopBit()) {
          if (i >= /*Msp430I2C1P.I2CP*/Msp430I2CP__0__TIMEOUT) {
              return EBUSY;
            }
          i++;
        }
      /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setTXStart();

      if (/*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_len == 1) {
          if (/*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_flags & I2C_STOP) {
              while (/*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getStartBit()) {
                  if (i >= /*Msp430I2C1P.I2CP*/Msp430I2CP__0__TIMEOUT) {
                      return EBUSY;
                    }
                  i++;
                }
              /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setTXStop();
            }
        }
    }
  else 
#line 152
    {
      /*Msp430I2C1P.I2CP*/Msp430I2CP__0__nextRead();
    }
  return SUCCESS;
}

# 76 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/I2CPacket.nc"
inline static error_t SimpleTMP102P__I2CBasicAddr__read(i2c_flags_t flags, uint16_t addr, uint8_t length, uint8_t * data){
#line 76
  unsigned char __nesc_result;
#line 76

#line 76
  __nesc_result = /*Msp430I2C1P.I2CP*/Msp430I2CP__0__I2CBasicAddr__read(flags, addr, length, data);
#line 76

#line 76
  return __nesc_result;
#line 76
}
#line 76
# 256 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline uint8_t HplMsp430UsciB1P__Usci__rx(void )
#line 256
{
  return HplMsp430UsciB1P__UCB1RXBUF;
}

# 115 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static uint8_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__rx(void ){
#line 115
  unsigned char __nesc_result;
#line 115

#line 115
  __nesc_result = HplMsp430UsciB1P__Usci__rx();
#line 115

#line 115
  return __nesc_result;
#line 115
}
#line 115
# 60 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void )
#line 60
{
  /* atomic removed: atomic calls only */
#line 61
  {
    unsigned char __nesc_temp = 
#line 61
    /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead == /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

#line 61
    return __nesc_temp;
  }
}

# 53 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 68 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void )
#line 68
{
  /* atomic removed: atomic calls only */
#line 69
  {
    if (/*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead != /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
        uint8_t id = /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead;

#line 72
        /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead = /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[/*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead];
        if (/*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead == /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
          /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail = /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
          }
#line 75
        /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[id] = /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 76
          id;

#line 76
          return __nesc_temp;
        }
      }
#line 78
    {
      unsigned char __nesc_temp = 
#line 78
      /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

#line 78
      return __nesc_temp;
    }
  }
}

# 70 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 59 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIORenP__34__IO__selectIOFunc(void )
#line 59
{
  /* atomic removed: atomic calls only */
#line 59
  * (volatile uint8_t * )51U &= ~(0x01 << 2);
}

# 99 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciB1P__USCL__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIORenP__34__IO__selectIOFunc();
#line 99
}
#line 99
# 59 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIORenP__33__IO__selectIOFunc(void )
#line 59
{
  /* atomic removed: atomic calls only */
#line 59
  * (volatile uint8_t * )51U &= ~(0x01 << 1);
}

# 99 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciB1P__USDA__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIORenP__33__IO__selectIOFunc();
#line 99
}
#line 99
# 281 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline void HplMsp430UsciB1P__Usci__disableI2C(void )
#line 281
{
  /* atomic removed: atomic calls only */
#line 282
  {
    HplMsp430UsciB1P__USDA__selectIOFunc();
    HplMsp430UsciB1P__USCL__selectIOFunc();
  }
}

# 155 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__disableI2C(void ){
#line 155
  HplMsp430UsciB1P__Usci__disableI2C();
#line 155
}
#line 155
# 102 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
static inline void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__ResourceConfigure__unconfigure(uint8_t id)
#line 102
{
  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__disableI2C();
}

# 218 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id)
#line 218
{
}

# 65 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(uint8_t arg_0x10bda6340){
#line 65
  switch (arg_0x10bda6340) {
#line 65
    case /*TestTmp102AppC.Temperature.I2C.UsciC*/Msp430UsciB1C__0__CLIENT_ID:
#line 65
      /*Msp430I2C1P.I2CP*/Msp430I2CP__0__ResourceConfigure__unconfigure(/*TestTmp102AppC.Temperature.I2C*/Msp430I2C1C__0__CLIENT_ID);
#line 65
      break;
#line 65
    default:
#line 65
      /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(arg_0x10bda6340);
#line 65
      break;
#line 65
    }
#line 65
}
#line 65
# 208 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void )
#line 208
{
}

# 46 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void ){
#line 46
  /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted();
#line 46
}
#line 46
# 50 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__toggle(void )
#line 50
{
#line 50
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 50
    * (volatile uint8_t * )49U ^= 0x01 << 5;
#line 50
    __nesc_atomic_end(__nesc_atomic); }
}

# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle(void ){
#line 58
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__toggle();
#line 58
}
#line 58
# 50 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void )
#line 50
{
#line 50
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle();
}

# 42 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__toggle(void ){
#line 42
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle();
#line 42
}
#line 42
# 114 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2Toggle(void )
#line 114
{
  LedsP__Led2__toggle();
  ;
#line 116
  ;
}

# 100 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Leds.nc"
inline static void TestTmp102C__Leds__led2Toggle(void ){
#line 100
  LedsP__Leds__led2Toggle();
#line 100
}
#line 100
# 191 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static uint16_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getSlaveAddress(void ){
#line 191
  unsigned int __nesc_result;
#line 191

#line 191
  __nesc_result = HplMsp430UsciB1P__Usci__getSlaveAddress();
#line 191

#line 191
  return __nesc_result;
#line 191
}
#line 191
# 67 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/TaskBasic.nc"
inline static error_t SimpleTMP102P__calculateTemp__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SimpleTMP102P__calculateTemp);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 278 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
static inline error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__default__isOwner(uint8_t id)
#line 278
{
#line 278
  return FAIL;
}

# 128 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
inline static bool /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__isOwner(uint8_t arg_0x10bc15020){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  switch (arg_0x10bc15020) {
#line 128
    case /*TestTmp102AppC.Temperature.I2C*/Msp430I2C1C__0__CLIENT_ID:
#line 128
      __nesc_result = /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(/*TestTmp102AppC.Temperature.I2C.UsciC*/Msp430UsciB1C__0__CLIENT_ID);
#line 128
      break;
#line 128
    default:
#line 128
      __nesc_result = /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__default__isOwner(arg_0x10bc15020);
#line 128
      break;
#line 128
    }
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
# 90 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
static inline uint8_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Resource__isOwner(uint8_t id)
#line 90
{
  return /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__isOwner(id);
}

# 128 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
inline static bool SimpleTMP102P__Resource__isOwner(void ){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  __nesc_result = /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Resource__isOwner(/*TestTmp102AppC.Temperature.I2C*/Msp430I2C1C__0__CLIENT_ID);
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
# 102 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/tmp102/SimpleTMP102P.nc"
static inline void SimpleTMP102P__I2CBasicAddr__readDone(error_t error, uint16_t addr, uint8_t length, uint8_t *data)
#line 102
{
  if (SimpleTMP102P__Resource__isOwner()) {
      uint16_t tmp;

#line 105
      for (tmp = 0; tmp < 0xffff; tmp++) ;
      SimpleTMP102P__Resource__release();
      tmp = data[0];
      tmp = tmp << 8;
      tmp = tmp + data[1];
      tmp = tmp >> 4;
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 111
        SimpleTMP102P__temp = tmp;
#line 111
        __nesc_atomic_end(__nesc_atomic); }
      SimpleTMP102P__calculateTemp__postTask();
    }
}

# 102 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/I2CPacket.nc"
inline static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__I2CBasicAddr__readDone(error_t error, uint16_t addr, uint8_t length, uint8_t * data){
#line 102
  SimpleTMP102P__I2CBasicAddr__readDone(error, addr, length, data);
#line 102
}
#line 102
# 252 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline void HplMsp430UsciB1P__Usci__tx(uint8_t data)
#line 252
{
  HplMsp430UsciB1P__UCB1TXBUF = data;
}

# 108 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__tx(uint8_t data){
#line 108
  HplMsp430UsciB1P__Usci__tx(data);
#line 108
}
#line 108
# 65 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
}

# 73 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 102 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop();
}

# 73 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 71 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void )
{
#line 72
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop();
}

# 78 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop();
#line 78
}
#line 78
# 64 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
}

# 109 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void ){
#line 109
  unsigned long __nesc_result;
#line 109

#line 109
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 96 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void )
{
#line 97
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow();
}

# 136 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void ){
#line 136
  unsigned long __nesc_result;
#line 136

#line 136
  __nesc_result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow();
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 159 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, TRUE);
}

# 73 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Timer.nc"
inline static void SimpleTMP102P__TimerSensor__startOneShot(uint32_t dt){
#line 73
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(1U, dt);
#line 73
}
#line 73
# 75 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/tmp102/SimpleTMP102P.nc"
static inline error_t SimpleTMP102P__Read__read(void )
#line 75
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 76
    P5DIR |= 0x01;
#line 76
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 77
    P5OUT |= 0x01;
#line 77
    __nesc_atomic_end(__nesc_atomic); }
  SimpleTMP102P__TimerSensor__startOneShot(100);

  return SUCCESS;
}

# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Read.nc"
inline static error_t TestTmp102C__TempSensor__read(void ){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = SimpleTMP102P__Read__read();
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 72 "TestTmp102C.nc"
static inline void TestTmp102C__TestTimer__fired(void )
#line 72
{
  TestTmp102C__TempSensor__read();
}

# 67 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 133 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void )
#line 133
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 134
    {
      if (/*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__resId == /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id) {
          if (/*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING) {
              /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
              {
                unsigned char __nesc_temp = 
#line 138
                SUCCESS;

                {
#line 138
                  __nesc_atomic_end(__nesc_atomic); 
#line 138
                  return __nesc_temp;
                }
              }
            }
          else {
#line 140
            if (/*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING) {
                /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
                /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
                {
                  unsigned char __nesc_temp = 
#line 143
                  SUCCESS;

                  {
#line 143
                    __nesc_atomic_end(__nesc_atomic); 
#line 143
                    return __nesc_temp;
                  }
                }
              }
            }
        }
    }
#line 149
    __nesc_atomic_end(__nesc_atomic); }
#line 147
  return FAIL;
}

#line 210
static inline void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void )
#line 210
{
  /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release();
}

# 73 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested(void ){
#line 73
  /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested();
#line 73
}
#line 73
# 64 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 64
{
  /* atomic removed: atomic calls only */
#line 65
  {
    unsigned char __nesc_temp = 
#line 65
    /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[id] != /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY || /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail == id;

#line 65
    return __nesc_temp;
  }
}

#line 82
static inline error_t /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id)
#line 82
{
  /* atomic removed: atomic calls only */
#line 83
  {
    if (!/*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(id)) {
        if (/*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead == /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
          /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead = id;
          }
        else {
#line 88
          /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[/*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail] = id;
          }
#line 89
        /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail = id;
        {
          unsigned char __nesc_temp = 
#line 90
          SUCCESS;

#line 90
          return __nesc_temp;
        }
      }
#line 92
    {
      unsigned char __nesc_temp = 
#line 92
      EBUSY;

#line 92
      return __nesc_temp;
    }
  }
}

# 79 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceQueue.nc"
inline static error_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(resource_client_id_t id){
#line 79
  unsigned char __nesc_result;
#line 79

#line 79
  __nesc_result = /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(id);
#line 79

#line 79
  return __nesc_result;
#line 79
}
#line 79
# 127 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/tmp102/SimpleTMP102P.nc"
static inline void SimpleTMP102P__ResourceRequested__requested(void )
#line 127
{
}

# 204 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(uint8_t id)
#line 204
{
}

# 53 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(uint8_t arg_0x10bda8020){
#line 53
  switch (arg_0x10bda8020) {
#line 53
    case /*TestTmp102AppC.Temperature.I2C.UsciC*/Msp430UsciB1C__0__CLIENT_ID:
#line 53
      SimpleTMP102P__ResourceRequested__requested();
#line 53
      break;
#line 53
    default:
#line 53
      /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(arg_0x10bda8020);
#line 53
      break;
#line 53
    }
#line 53
}
#line 53
# 77 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(uint8_t id)
#line 77
{
  /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(/*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 79
    {
      if (/*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) {
          /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING;
          /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = id;
        }
      else {
#line 84
        if (/*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId == id) {
            {
              unsigned char __nesc_temp = 
#line 85
              SUCCESS;

              {
#line 85
                __nesc_atomic_end(__nesc_atomic); 
#line 85
                return __nesc_temp;
              }
            }
          }
        else 
#line 87
          {
            unsigned char __nesc_temp = 
#line 87
            /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(id);

            {
#line 87
              __nesc_atomic_end(__nesc_atomic); 
#line 87
              return __nesc_temp;
            }
          }
        }
    }
#line 91
    __nesc_atomic_end(__nesc_atomic); }
#line 89
  /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested();
  return SUCCESS;
}

# 110 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
static inline error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__default__request(uint8_t id)
#line 110
{
#line 110
  return FAIL;
}

# 88 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
inline static error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__request(uint8_t arg_0x10bc15020){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  switch (arg_0x10bc15020) {
#line 88
    case /*TestTmp102AppC.Temperature.I2C*/Msp430I2C1C__0__CLIENT_ID:
#line 88
      __nesc_result = /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(/*TestTmp102AppC.Temperature.I2C.UsciC*/Msp430UsciB1C__0__CLIENT_ID);
#line 88
      break;
#line 88
    default:
#line 88
      __nesc_result = /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__default__request(arg_0x10bc15020);
#line 88
      break;
#line 88
    }
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 86 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
static inline error_t /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Resource__request(uint8_t id)
#line 86
{
  return /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciResource__request(id);
}

# 88 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Resource.nc"
inline static error_t SimpleTMP102P__Resource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Resource__request(/*TestTmp102AppC.Temperature.I2C*/Msp430I2C1C__0__CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 83 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/tmp102/SimpleTMP102P.nc"
static inline void SimpleTMP102P__TimerSensor__fired(void )
#line 83
{
  SimpleTMP102P__Resource__request();
}

static inline void SimpleTMP102P__TimerFail__fired(void )
#line 87
{
  SimpleTMP102P__Read__readDone(SUCCESS, 0);
}

# 204 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 83 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0x10bae15d8){
#line 83
  switch (arg_0x10bae15d8) {
#line 83
    case 0U:
#line 83
      TestTmp102C__TestTimer__fired();
#line 83
      break;
#line 83
    case 1U:
#line 83
      SimpleTMP102P__TimerSensor__fired();
#line 83
      break;
#line 83
    case 2U:
#line 83
      SimpleTMP102P__TimerFail__fired();
#line 83
      break;
#line 83
    default:
#line 83
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0x10bae15d8);
#line 83
      break;
#line 83
    }
#line 83
}
#line 83
# 67 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 103 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(t0, dt);
}

#line 93
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 94
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(t0, dt, TRUE);
}

# 129 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt){
#line 129
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(t0, dt);
#line 129
}
#line 129
# 91 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 93
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type __nesc_temp = 
#line 93
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

      {
#line 93
        __nesc_atomic_end(__nesc_atomic); 
#line 93
        return __nesc_temp;
      }
    }
#line 95
    __nesc_atomic_end(__nesc_atomic); }
}

# 116 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void ){
#line 116
  unsigned long __nesc_result;
#line 116

#line 116
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm();
#line 116

#line 116
  return __nesc_result;
#line 116
}
#line 116
# 139 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow());
}

# 83 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void ){
#line 83
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired();
#line 83
}
#line 83
# 69 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 57 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4306 {
#line 57
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t f;
#line 57
    uint16_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(x);
}

#line 105
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )386U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl();
}

# 47 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare();
#line 47
}
#line 47
# 53 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 55 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void )
#line 55
{
  memset(/*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ, /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY, sizeof /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ);
  return SUCCESS;
}

# 62 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = /*Msp430UsciShareB1P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init();
#line 62
  __nesc_result = ecombine(__nesc_result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init());
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 154 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, FALSE);
}

# 64 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/Timer.nc"
inline static void TestTmp102C__TestTimer__startPeriodic(uint32_t dt){
#line 64
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(0U, dt);
#line 64
}
#line 64
# 56 "TestTmp102C.nc"
static inline void TestTmp102C__printTitles(void )
#line 56
{
  {
#line 57
    sprintf(debugbuf, "\n\n");
#line 57
    writedebug();
  }
#line 57
  ;
  {
#line 58
    sprintf(debugbuf, "   ###############################\n");
#line 58
    writedebug();
  }
#line 58
  ;
  {
#line 59
    sprintf(debugbuf, "   #                             #\n");
#line 59
    writedebug();
  }
#line 59
  ;
  {
#line 60
    sprintf(debugbuf, "   #          TMP102 TEST        #\n");
#line 60
    writedebug();
  }
#line 60
  ;
  {
#line 61
    sprintf(debugbuf, "   #                             #\n");
#line 61
    writedebug();
  }
#line 61
  ;
  {
#line 62
    sprintf(debugbuf, "   ###############################\n");
#line 62
    writedebug();
  }
#line 62
  ;
  {
#line 63
    sprintf(debugbuf, "\n");
#line 63
    writedebug();
  }
#line 63
  ;
}

# 126 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/PrintfUART.h"
static inline void printfUART_init_private()
{
#line 197
  P3SEL |= 0x30;
  UCA0CTL1 |= 0x80;
  UCA0BR0 = 0x45;
  UCA0BR1 = 0x00;
  UCA0MCTL = 0x04 + 0x02;
  UCA0CTL1 &= ~0x01;
}

# 66 "TestTmp102C.nc"
static inline void TestTmp102C__Boot__booted(void )
#line 66
{
  {
#line 67
    { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 67
      printfUART_init_private();
#line 67
      __nesc_atomic_end(__nesc_atomic); }
  }
#line 67
  ;
  TestTmp102C__printTitles();
  TestTmp102C__TestTimer__startPeriodic(1024);
}

# 60 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 60
  TestTmp102C__Boot__booted();
#line 60
}
#line 60
# 66 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/tmp102/SimpleTMP102P.nc"
static inline void SimpleTMP102P__calculateTemp__runTask(void )
#line 66
{
  uint16_t tmp = SimpleTMP102P__temp;




  SimpleTMP102P__Read__readDone(SUCCESS, tmp);
}

# 175 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id)
{
}

# 75 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/TaskBasic.nc"
inline static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x10b4ee328){
#line 75
  switch (arg_0x10b4ee328) {
#line 75
    case /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired:
#line 75
      /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask();
#line 75
      break;
#line 75
    case /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer:
#line 75
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask();
#line 75
      break;
#line 75
    case SimpleTMP102P__calculateTemp:
#line 75
      SimpleTMP102P__calculateTemp__runTask();
#line 75
      break;
#line 75
    case /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask:
#line 75
      /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask();
#line 75
      break;
#line 75
    default:
#line 75
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x10b4ee328);
#line 75
      break;
#line 75
    }
#line 75
}
#line 75
# 402 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
  __dint();
  __nop();
}

# 66 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/platforms/z1/chips/msp430/timer/Msp430ClockP.nc"
static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void )
#line 66
{
  return MSP430_POWER_LPM3;
}

# 62 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Msp430ClockP__McuPowerOverride__lowestState();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 74 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/platforms/z1/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC__getPowerState(void )
#line 74
{
  mcu_power_t pState = MSP430_POWER_LPM3;





  if (((((((
#line 77
  TACCTL0 & 0x0010 || TACCTL1 & 0x0010) || TACCTL2 & 0x0010) && (
  TACTL & 0x0300) == 0x0200) || (
  UCA0CTL1 & 0xC0) != 0x00) || (
  UCA1CTL1 & 0xC0) != 0x00) || (
  UCB0CTL1 & 0xC0) != 0x00) || (
  UCB1CTL1 & 0xC0) != 0x00) {

    pState = MSP430_POWER_LPM1;
    }


  if (ADC12CTL0 & 0x010) {
      if (ADC12CTL1 & 0x0010) {

          if (ADC12CTL1 & 0x0008) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 94
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 95
        if (ADC12CTL1 & 0x0400 && (TACTL & 0x0300) == 0x0200) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

# 390 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/msp430hardware.h"
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 390
{
  return m1 < m2 ? m1 : m2;
}

# 107 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/platforms/z1/chips/msp430/McuSleepC.nc"
static inline void McuSleepC__computePowerState(void )
#line 107
{
  McuSleepC__powerState = mcombine(McuSleepC__getPowerState(), 
  McuSleepC__McuPowerOverride__lowestState());
}

static inline void McuSleepC__McuSleep__sleep(void )
#line 112
{
  uint16_t temp;

#line 114
  if (McuSleepC__dirty) {
      McuSleepC__computePowerState();
    }

  temp = McuSleepC__msp430PowerBits[McuSleepC__powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
}

# 76 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP__McuSleep__sleep(void ){
#line 76
  McuSleepC__McuSleep__sleep();
#line 76
}
#line 76
# 78 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP__popTask(void )
{
  if (SchedulerBasicP__m_head != SchedulerBasicP__NO_TASK) 
    {
      uint8_t id = SchedulerBasicP__m_head;

#line 83
      SchedulerBasicP__m_head = SchedulerBasicP__m_next[SchedulerBasicP__m_head];
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
        }
      SchedulerBasicP__m_next[id] = SchedulerBasicP__NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP__NO_TASK;
    }
}

#line 149
static inline void SchedulerBasicP__Scheduler__taskLoop(void )
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP__popTask()) == SchedulerBasicP__NO_TASK) 
            {
              SchedulerBasicP__McuSleep__sleep();
            }
        }
#line 161
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP__TaskBasic__runTask(nextTask);
    }
}

# 72 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__taskLoop(void ){
#line 72
  SchedulerBasicP__Scheduler__taskLoop();
#line 72
}
#line 72
# 99 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciAB1RawInterruptsP.nc"
static inline void HplMsp430UsciAB1RawInterruptsP__UsciA__default__rxDone(uint8_t temp)
#line 99
{
  return;
}

# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciRawInterrupts.nc"
inline static void HplMsp430UsciAB1RawInterruptsP__UsciA__rxDone(uint8_t data){
#line 58
  HplMsp430UsciAB1RawInterruptsP__UsciA__default__rxDone(data);
#line 58
}
#line 58
# 59 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
inline static void HplMsp430UsciB1P__Interrupts__rxDone(uint8_t data){
#line 59
  /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__rxDone(data);
#line 59
}
#line 59
# 84 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline void HplMsp430UsciB1P__UsciRawInterrupts__rxDone(uint8_t temp)
#line 84
{
  HplMsp430UsciB1P__Interrupts__rxDone(temp);
}

# 58 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciRawInterrupts.nc"
inline static void HplMsp430UsciAB1RawInterruptsP__UsciB__rxDone(uint8_t data){
#line 58
  HplMsp430UsciB1P__UsciRawInterrupts__rxDone(data);
#line 58
}
#line 58
# 350 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline bool HplMsp430UsciB1P__Usci__getTransmitReceiveMode(void )
#line 350
{
#line 350
  return HplMsp430UsciB1P__UCB1CTL1 & 0x10;
}

# 186 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static bool /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getTransmitReceiveMode(void ){
#line 186
  unsigned char __nesc_result;
#line 186

#line 186
  __nesc_result = HplMsp430UsciB1P__Usci__getTransmitReceiveMode();
#line 186

#line 186
  return __nesc_result;
#line 186
}
#line 186
# 211 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline void HplMsp430UsciB1P__Usci__clrRxIntr(void )
#line 211
{
  HplMsp430UsciB1P__UC1IFG &= ~0x04;
}

# 100 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__clrRxIntr(void ){
#line 100
  HplMsp430UsciB1P__Usci__clrRxIntr();
#line 100
}
#line 100
# 262 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
static inline void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Interrupts__rxDone(uint8_t data)
#line 262
{
  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__clrRxIntr();
  if (/*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getTransmitReceiveMode()) {
    /*Msp430I2C1P.I2CP*/Msp430I2CP__0__nextWrite();
    }
  else {
#line 267
    /*Msp430I2C1P.I2CP*/Msp430I2CP__0__nextRead();
    }
}

# 60 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 60
{
}

# 59 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
inline static void /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__Interrupts__rxDone(uint8_t arg_0x10bd75108, uint8_t data){
#line 59
  switch (arg_0x10bd75108) {
#line 59
    case /*TestTmp102AppC.Temperature.I2C.UsciC*/Msp430UsciB1C__0__CLIENT_ID:
#line 59
      /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Interrupts__rxDone(data);
#line 59
      break;
#line 59
    default:
#line 59
      /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__rxDone(arg_0x10bd75108, data);
#line 59
      break;
#line 59
    }
#line 59
}
#line 59
# 95 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciAB1RawInterruptsP.nc"
static inline void HplMsp430UsciAB1RawInterruptsP__UsciA__default__txDone(void )
#line 95
{
  return;
}

# 53 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciRawInterrupts.nc"
inline static void HplMsp430UsciAB1RawInterruptsP__UsciA__txDone(void ){
#line 53
  HplMsp430UsciAB1RawInterruptsP__UsciA__default__txDone();
#line 53
}
#line 53
# 98 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__userId(void ){
#line 98
  unsigned char __nesc_result;
#line 98

#line 98
  __nesc_result = /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 207 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline void HplMsp430UsciB1P__Usci__clrTxIntr(void )
#line 207
{
  HplMsp430UsciB1P__UC1IFG &= ~0x08;
}

# 99 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__clrTxIntr(void ){
#line 99
  HplMsp430UsciB1P__Usci__clrTxIntr();
#line 99
}
#line 99
# 254 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
static inline void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Interrupts__txDone(void )
#line 254
{
  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__clrTxIntr();
  if (/*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getTransmitReceiveMode()) {
    /*Msp430I2C1P.I2CP*/Msp430I2CP__0__nextWrite();
    }
  else {
#line 259
    /*Msp430I2C1P.I2CP*/Msp430I2CP__0__nextRead();
    }
}

# 59 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__txDone(uint8_t id)
#line 59
{
}

# 54 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
inline static void /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__Interrupts__txDone(uint8_t arg_0x10bd75108){
#line 54
  switch (arg_0x10bd75108) {
#line 54
    case /*TestTmp102AppC.Temperature.I2C.UsciC*/Msp430UsciB1C__0__CLIENT_ID:
#line 54
      /*Msp430I2C1P.I2CP*/Msp430I2CP__0__Interrupts__txDone();
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__txDone(arg_0x10bd75108);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 90 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__inUse(void ){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse();
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 49 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__txDone(void )
#line 49
{
  if (/*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__Interrupts__txDone(/*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__userId());
    }
}

# 54 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
inline static void HplMsp430UsciB1P__Interrupts__txDone(void ){
#line 54
  /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__txDone();
#line 54
}
#line 54
# 88 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static inline void HplMsp430UsciB1P__UsciRawInterrupts__txDone(void )
#line 88
{
  HplMsp430UsciB1P__Interrupts__txDone();
}

# 53 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciRawInterrupts.nc"
inline static void HplMsp430UsciAB1RawInterruptsP__UsciB__txDone(void ){
#line 53
  HplMsp430UsciB1P__UsciRawInterrupts__txDone();
#line 53
}
#line 53
# 422 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/msp430hardware.h"
  __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (__read_status_register() & 0x0008) != 0;

#line 425
  __nesc_disable_interrupt();
   __asm volatile ("" :  :  : "memory");
  return result;
}

  void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
   __asm volatile ("" :  :  : "memory");
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 11 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0032)))  void sig_TIMERA0_VECTOR(void )
#line 11
{
#line 11
  Msp430TimerCommonP__VectorTimerA0__fired();
}

# 180 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired();
    }
}

#line 180
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired();
    }
}

#line 180
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired();
    }
}

# 12 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0030)))  void sig_TIMERA1_VECTOR(void )
#line 12
{
#line 12
  Msp430TimerCommonP__VectorTimerA1__fired();
}

#line 13
__attribute((wakeup)) __attribute((interrupt(0x003A)))  void sig_TIMERB0_VECTOR(void )
#line 13
{
#line 13
  Msp430TimerCommonP__VectorTimerB0__fired();
}

# 146 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n)
{
}

# 39 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x10b5f3458){
#line 39
  switch (arg_0x10b5f3458) {
#line 39
    case 0:
#line 39
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired();
#line 39
      break;
#line 39
    case 1:
#line 39
      /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired();
#line 39
      break;
#line 39
    case 2:
#line 39
      /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired();
#line 39
      break;
#line 39
    case 3:
#line 39
      /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired();
#line 39
      break;
#line 39
    case 4:
#line 39
      /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired();
#line 39
      break;
#line 39
    case 5:
#line 39
      /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired();
#line 39
      break;
#line 39
    case 6:
#line 39
      /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired();
#line 39
      break;
#line 39
    case 7:
#line 39
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired();
#line 39
      break;
#line 39
    default:
#line 39
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x10b5f3458);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 170 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 172
    {
#line 172
      {
        unsigned char __nesc_temp = 
#line 172
        SchedulerBasicP__pushTask(id) ? SUCCESS : EBUSY;

        {
#line 172
          __nesc_atomic_end(__nesc_atomic); 
#line 172
          return __nesc_temp;
        }
      }
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
}

# 107 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
#line 109
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type expires;
#line 109
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 132
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )remaining << 5);
}

# 80 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/TransformCounterC.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type rv = 0;

#line 83
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();

#line 87
      if (/*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT;

#line 101
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC__0__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 62 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void )
{




  if (1) {
      /* atomic removed: atomic calls only */
#line 69
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )400U;

#line 72
        do {
#line 72
            t0 = t1;
#line 72
            t1 = * (volatile uint16_t * )400U;
          }
        while (
#line 72
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 73
          t1;

#line 73
          return __nesc_temp;
        }
      }
    }
  else 
#line 76
    {
      return * (volatile uint16_t * )400U;
    }
}

# 14 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0038)))  void sig_TIMERB1_VECTOR(void )
#line 14
{
#line 14
  Msp430TimerCommonP__VectorTimerB1__fired();
}

# 63 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/RealMainP.nc"
  int main(void )
#line 63
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 71
      ;

      RealMainP__Scheduler__init();





      RealMainP__PlatformInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;





      RealMainP__SoftwareInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;
    }
#line 88
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP__Boot__booted();


  RealMainP__Scheduler__taskLoop();




  return -1;
}

# 134 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP__Scheduler__runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 138
  {
    nextTask = SchedulerBasicP__popTask();
    if (nextTask == SchedulerBasicP__NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 142
          FALSE;

#line 142
          return __nesc_temp;
        }
      }
  }
#line 145
  SchedulerBasicP__TaskBasic__runTask(nextTask);
  return TRUE;
}

# 190 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
static void /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void )
#line 190
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 191
    {
      /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
      /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
    }
#line 194
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(/*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__resId);
}

# 228 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__nextWrite(void )
#line 228
{
  uint16_t i = 0;





  if (/*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_pos == /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_len && /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_flags & I2C_STOP) {
      /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setTXStop();
      while (/*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getStopBit()) {
          if (i >= /*Msp430I2C1P.I2CP*/Msp430I2CP__0__TIMEOUT) {
              /*Msp430I2C1P.I2CP*/Msp430I2CP__0__signalDone(EBUSY);
              return;
            }
          i++;
        }
      /*Msp430I2C1P.I2CP*/Msp430I2CP__0__signalDone(SUCCESS);
    }
  else 
#line 245
    {
      if (/*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getUstat() == ((0x10 | 0x08) | 0x40)) {
          /*Msp430I2C1P.I2CP*/Msp430I2CP__0__I2CBasicAddr__writeDone(FAIL, /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getSlaveAddress(), /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_len, /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_buf);
          return;
        }
      /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__tx(/*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_buf[/*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_pos++]);
    }
}

#line 270
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__signalDone(error_t error)
#line 270
{
  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__clrIntr();
  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__disableIntr();
  if (/*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getTransmitReceiveMode()) {
    /*Msp430I2C1P.I2CP*/Msp430I2CP__0__I2CBasicAddr__writeDone(error, /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getSlaveAddress(), /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_len, /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_buf);
    }
  else {
#line 276
    /*Msp430I2C1P.I2CP*/Msp430I2CP__0__I2CBasicAddr__readDone(error, /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getSlaveAddress(), /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_len, /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_buf);
    }
}

# 116 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/tmp102/SimpleTMP102P.nc"
static void SimpleTMP102P__I2CBasicAddr__writeDone(error_t error, uint16_t addr, uint8_t length, uint8_t *data)
#line 116
{
  if (SimpleTMP102P__Resource__isOwner()) {
      error_t e;

#line 119
      e = SimpleTMP102P__I2CBasicAddr__read(I2C_START | I2C_STOP, 0x48, 2, SimpleTMP102P__temperaturebuff);
      if (e) {
          SimpleTMP102P__Resource__release();
          SimpleTMP102P__Read__readDone(error, 0);
        }
    }
}

# 177 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
static bool /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id)
#line 177
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 178
    {
      if (/*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__resId == id && /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) {
          unsigned char __nesc_temp = 
#line 179
          TRUE;

          {
#line 179
            __nesc_atomic_end(__nesc_atomic); 
#line 179
            return __nesc_temp;
          }
        }
      else 
#line 180
        {
          unsigned char __nesc_temp = 
#line 180
          FALSE;

          {
#line 180
            __nesc_atomic_end(__nesc_atomic); 
#line 180
            return __nesc_temp;
          }
        }
    }
#line 183
    __nesc_atomic_end(__nesc_atomic); }
}

# 199 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430I2CP.nc"
static void /*Msp430I2C1P.I2CP*/Msp430I2CP__0__nextRead(void )
#line 199
{
  uint16_t i = 0;





  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_buf[/*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_pos++] = /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__rx();
  if (/*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_pos == /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_len - 1) {
      if (/*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_flags & I2C_STOP) {
          /*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__setTXStop();
        }
    }
  if (/*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_pos == /*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_len) {
      if (/*Msp430I2C1P.I2CP*/Msp430I2CP__0__m_flags & I2C_STOP) {
          while (!/*Msp430I2C1P.I2CP*/Msp430I2CP__0__UsciB__getStopBit()) {
              if (i >= /*Msp430I2C1P.I2CP*/Msp430I2CP__0__TIMEOUT) {
                  /*Msp430I2C1P.I2CP*/Msp430I2CP__0__signalDone(EBUSY);
                  return;
                }
              i++;
            }
          /*Msp430I2C1P.I2CP*/Msp430I2CP__0__signalDone(SUCCESS);
        }
      else 
#line 222
        {
          /*Msp430I2C1P.I2CP*/Msp430I2CP__0__signalDone(SUCCESS);
        }
    }
}

# 111 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
static error_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id)
#line 111
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
    {
      if (/*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY && /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
          if (/*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty() == FALSE) {
              /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue();
              /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;
              /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING;
              /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
              /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
            }
          else {
              /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
              /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
              /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
              /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted();
            }
          {
            unsigned char __nesc_temp = 
#line 127
            SUCCESS;

            {
#line 127
              __nesc_atomic_end(__nesc_atomic); 
#line 127
              return __nesc_temp;
            }
          }
        }
    }
#line 131
    __nesc_atomic_end(__nesc_atomic); }
#line 130
  return FAIL;
}

# 76 "TestTmp102C.nc"
static void TestTmp102C__TempSensor__readDone(error_t error, uint16_t data)
#line 76
{
  if (error == SUCCESS) {
      TestTmp102C__Leds__led2Toggle();
      if (data > 2047) {
#line 79
        data -= 1 << 12;
        }
#line 80
      data *= 0.625;
      {
#line 81
        sprintf(debugbuf, "Temp: %2d.%1.2d\n", data / 10, data >> 2);
#line 81
        writedebug();
      }
#line 81
      ;
    }
}

# 349 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/PrintfUART.h"
static void writedebug()
{
  uint16_t i = 0;

  while (debugbuf[i] != '\0' && i < 256) 
    UARTPutChar(debugbuf[i++]);
}

#line 319
static void UARTPutChar(char c)
{
  if (c == '\n') {
    UARTPutChar('\r');
    }










  while (!(IFG2 & 0x02)) ;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 335
    UCA0TXBUF = c;
#line 335
    __nesc_atomic_end(__nesc_atomic); }
}

# 353 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB1P.nc"
static uint16_t HplMsp430UsciB1P__Usci__getSlaveAddress(void )
#line 353
{
#line 353
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 353
    {
#line 353
      {
        unsigned int __nesc_temp = 
#line 353
        UCB1I2CSA;

        {
#line 353
          __nesc_atomic_end(__nesc_atomic); 
#line 353
          return __nesc_temp;
        }
      }
    }
#line 356
    __nesc_atomic_end(__nesc_atomic); }
}

# 100 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint16_t num;

  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop();

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(now);
        }
      else {
#line 135
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(now, min_remaining);
        }
    }
}

#line 73
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now)
{
  uint16_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 90
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(num);
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

#line 144
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

#line 147
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 147 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
    }
#line 154
    __nesc_atomic_end(__nesc_atomic); }
}

# 74 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/lib/timer/AlarmToTimerC.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt, FALSE);
    }
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired();
}

# 54 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciAB1RawInterruptsP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0022)))  void sig_USCIAB1RX_VECTOR(void )
#line 54
{
  uint8_t temp;


  if (UC1IFG & 0x01 && UC1IE & 0x01) {
      temp = UCA1RXBUF;
      HplMsp430UsciAB1RawInterruptsP__UsciA__rxDone(temp);
    }




  if (UC1IFG & 0x04 && UC1IE & 0x04 && UCB1CTL0 & 0x01 && (UCB1CTL0 & (0x04 | 0x02)) != 0x06) {
      temp = UCB1RXBUF;
      HplMsp430UsciAB1RawInterruptsP__UsciB__rxDone(temp);
    }
}

# 54 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/Msp430UsciShareP.nc"
static void /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__Interrupts__rxDone(/*Msp430UsciShareB1P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__userId(), data);
    }
}

# 153 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/system/ArbiterP.nc"
static bool /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void )
#line 153
{
  /* atomic removed: atomic calls only */
#line 154
  {
    if (/*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 156
        FALSE;

#line 156
        return __nesc_temp;
      }
  }
#line 158
  return TRUE;
}






static uint8_t /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void )
#line 166
{
  /* atomic removed: atomic calls only */
#line 167
  {
    if (/*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__state != /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 169
        /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;

#line 169
        return __nesc_temp;
      }
#line 170
    {
      unsigned char __nesc_temp = 
#line 170
      /*Msp430UsciShareB1P.ArbiterC.Arbiter*/ArbiterP__0__resId;

#line 170
      return __nesc_temp;
    }
  }
}

# 73 "/Users/Xavier/Documents/tinyOS/tinyOSsource/tos/chips/msp430/x2xxx/usci/HplMsp430UsciAB1RawInterruptsP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0020)))  void sig_USCIAB1TX_VECTOR(void )
#line 73
{
  uint8_t temp;

  if (UC1IFG & 0x02 && UC1IE & 0x02) {
      HplMsp430UsciAB1RawInterruptsP__UsciA__txDone();
    }




  if (UC1IFG & 0x04 && UC1IE & 0x04 && UCB1CTL0 & 0x01 && (UCB1CTL0 & (0x04 | 0x02)) == 0x06) {
      temp = UCB1RXBUF;
      HplMsp430UsciAB1RawInterruptsP__UsciB__rxDone(temp);
    }


  if (UC1IFG & 0x08 && UC1IE & 0x08) {
      HplMsp430UsciAB1RawInterruptsP__UsciB__txDone();
    }
}

