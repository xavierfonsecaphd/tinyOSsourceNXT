#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 150 "/opt/local/lib/gcc/arm-elf/4.7.3/include/stddef.h" 3
typedef long int ptrdiff_t;
#line 213
typedef long unsigned int size_t;
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
# 41 "/opt/local/lib/gcc/arm-elf/4.7.3/../../../../arm-elf/include/stdint.h" 3
typedef signed char int8_t;
typedef unsigned char uint8_t;




typedef signed char int_least8_t;
typedef unsigned char uint_least8_t;




typedef signed short int16_t;
typedef unsigned short uint16_t;
#line 67
typedef int16_t int_least16_t;
typedef uint16_t uint_least16_t;










typedef signed long int32_t;
typedef unsigned long uint32_t;
#line 97
typedef int32_t int_least32_t;
typedef uint32_t uint_least32_t;
#line 119
typedef signed long long int64_t;
typedef unsigned long long uint64_t;








typedef int64_t int_least64_t;
typedef uint64_t uint_least64_t;
#line 159
typedef signed int int_fast8_t;
typedef unsigned int uint_fast8_t;




typedef signed int int_fast16_t;
typedef unsigned int uint_fast16_t;




typedef signed int int_fast32_t;
typedef unsigned int uint_fast32_t;
#line 213
typedef int_least64_t int_fast64_t;
typedef uint_least64_t uint_fast64_t;







typedef long long int intmax_t;








typedef long long unsigned int uintmax_t;
#line 243
typedef signed long int intptr_t;
typedef unsigned long int uintptr_t;
# 273 "/opt/local/lib/gcc/arm-elf/4.7.3/../../../../arm-elf/include/inttypes.h" 3
#line 270
typedef struct __nesc_unnamed4242 {
  intmax_t quot;
  intmax_t rem;
} imaxdiv_t;
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
# 26 "/opt/local/lib/gcc/arm-elf/4.7.3/../../../../arm-elf/include/machine/_default_types.h" 3
typedef signed char __int8_t;
typedef unsigned char __uint8_t;








typedef signed short __int16_t;
typedef unsigned short __uint16_t;








typedef __int16_t __int_least16_t;
typedef __uint16_t __uint_least16_t;










typedef signed int __int32_t;
typedef unsigned int __uint32_t;
#line 76
typedef __int32_t __int_least32_t;
typedef __uint32_t __uint_least32_t;
#line 99
typedef signed long long __int64_t;
typedef unsigned long long __uint64_t;
# 6 "/opt/local/lib/gcc/arm-elf/4.7.3/../../../../arm-elf/include/sys/lock.h" 3
typedef int _LOCK_T;
typedef int _LOCK_RECURSIVE_T;
# 16 "/opt/local/lib/gcc/arm-elf/4.7.3/../../../../arm-elf/include/sys/_types.h" 3
typedef long _off_t;







typedef short __dev_t;




typedef unsigned short __uid_t;


typedef unsigned short __gid_t;



__extension__ 
#line 36
typedef long long _off64_t;







typedef long _fpos_t;
#line 56
typedef int _ssize_t;
# 354 "/opt/local/lib/gcc/arm-elf/4.7.3/include/stddef.h" 3
typedef unsigned int wint_t;
# 75 "/opt/local/lib/gcc/arm-elf/4.7.3/../../../../arm-elf/include/sys/_types.h" 3
#line 67
typedef struct __nesc_unnamed4243 {

  int __count;
  union __nesc_unnamed4244 {

    wint_t __wch;
    unsigned char __wchb[4];
  } __value;
} _mbstate_t;



typedef _LOCK_RECURSIVE_T _flock_t;




typedef void *_iconv_t;
# 21 "/opt/local/lib/gcc/arm-elf/4.7.3/../../../../arm-elf/include/sys/reent.h" 3
typedef unsigned long __ULong;
#line 37
struct _reent;






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







struct _on_exit_args {
  void *_fnargs[32];
  void *_dso_handle[32];

  __ULong _fntypes;


  __ULong _is_cxa;
};









struct _atexit {
  struct _atexit *_next;
  int _ind;

  void (*_fns[32])(void );
  struct _on_exit_args _on_exit_args;
};









struct __sbuf {
  unsigned char *_base;
  int _size;
};
#line 169
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;






  void *_cookie;

  int (*_read)(struct _reent *arg_0x1040397a8, void *arg_0x104039a48, char *arg_0x104039ce8, int arg_0x104038020);

  int (*_write)(struct _reent *arg_0x104038730, void *arg_0x1040389d0, const char *arg_0x104038ca8, int arg_0x10403f020);

  _fpos_t (*_seek)(struct _reent *arg_0x10403f770, void *arg_0x10403fa10, _fpos_t arg_0x10403fcc0, int arg_0x10403d020);
  int (*_close)(struct _reent *arg_0x10403d730, void *arg_0x10403d9d0);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;


  struct _reent *_data;



  _flock_t _lock;

  _mbstate_t _mbstate;
  int _flags2;
};
#line 273
typedef struct __sFILE __FILE;



struct _glue {

  struct _glue *_next;
  int _niobs;
  __FILE *_iobs;
};
#line 305
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};
#line 580
struct _reent {

  int _errno;




  __FILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *arg_0x104045b18);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4245 {

    struct __nesc_unnamed4246 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
      _mbstate_t _mblen_state;
      _mbstate_t _mbtowc_state;
      _mbstate_t _wctomb_state;
      char _l64a_buf[8];
      char _signal_buf[24];
      int _getdate_err;
      _mbstate_t _mbrlen_state;
      _mbstate_t _mbrtowc_state;
      _mbstate_t _mbsrtowcs_state;
      _mbstate_t _wcrtomb_state;
      _mbstate_t _wcsrtombs_state;
      int _h_errno;
    } _reent;



    struct __nesc_unnamed4247 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x104053060);




  struct _glue __sglue;
  __FILE __sf[3];
};
#line 818
struct _reent;
struct _reent;
# 27 "/opt/local/lib/gcc/arm-elf/4.7.3/../../../../arm-elf/include/string.h" 3
void *memset(void *arg_0x104064bf0, int arg_0x104064e58, size_t arg_0x104063140);
# 33 "/opt/local/lib/gcc/arm-elf/4.7.3/../../../../arm-elf/include/stdlib.h" 3
#line 29
typedef struct __nesc_unnamed4248 {

  int quot;
  int rem;
} div_t;





#line 35
typedef struct __nesc_unnamed4249 {

  long quot;
  long rem;
} ldiv_t;






#line 42
typedef struct __nesc_unnamed4250 {

  long long int quot;
  long long int rem;
} lldiv_t;
# 14 "/opt/local/lib/gcc/arm-elf/4.7.3/../../../../arm-elf/include/math.h" 3
union __dmath {

  double d;
  __ULong i[2];
};

union __fmath {

  float f;
  __ULong i[1];
};


union __ldmath {

  long double ld;
  __ULong i[4];
};
#line 148
typedef float float_t;
typedef double double_t;
#line 499
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 554
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 25 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4251 {
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
# 51 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/types/TinyError.h"
enum __nesc_unnamed4252 {
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
# 45 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/platforms/nxtmote/AT91SAM7S256.h"
typedef volatile unsigned int AT91_REG;
#line 171
#line 50
typedef struct _AT91S_SYS {
  AT91_REG AIC_SMR[32];
  AT91_REG AIC_SVR[32];
  AT91_REG AIC_IVR;
  AT91_REG AIC_FVR;
  AT91_REG AIC_ISR;
  AT91_REG AIC_IPR;
  AT91_REG AIC_IMR;
  AT91_REG AIC_CISR;
  AT91_REG Reserved0[2];
  AT91_REG AIC_IECR;
  AT91_REG AIC_IDCR;
  AT91_REG AIC_ICCR;
  AT91_REG AIC_ISCR;
  AT91_REG AIC_EOICR;
  AT91_REG AIC_SPU;
  AT91_REG AIC_DCR;
  AT91_REG Reserved1[1];
  AT91_REG AIC_FFER;
  AT91_REG AIC_FFDR;
  AT91_REG AIC_FFSR;
  AT91_REG Reserved2[45];
  AT91_REG DBGU_CR;
  AT91_REG DBGU_MR;
  AT91_REG DBGU_IER;
  AT91_REG DBGU_IDR;
  AT91_REG DBGU_IMR;
  AT91_REG DBGU_CSR;
  AT91_REG DBGU_RHR;
  AT91_REG DBGU_THR;
  AT91_REG DBGU_BRGR;
  AT91_REG Reserved3[7];
  AT91_REG DBGU_CIDR;
  AT91_REG DBGU_EXID;
  AT91_REG DBGU_FNTR;
  AT91_REG Reserved4[45];
  AT91_REG DBGU_RPR;
  AT91_REG DBGU_RCR;
  AT91_REG DBGU_TPR;
  AT91_REG DBGU_TCR;
  AT91_REG DBGU_RNPR;
  AT91_REG DBGU_RNCR;
  AT91_REG DBGU_TNPR;
  AT91_REG DBGU_TNCR;
  AT91_REG DBGU_PTCR;
  AT91_REG DBGU_PTSR;
  AT91_REG Reserved5[54];
  AT91_REG PIOA_PER;
  AT91_REG PIOA_PDR;
  AT91_REG PIOA_PSR;
  AT91_REG Reserved6[1];
  AT91_REG PIOA_OER;
  AT91_REG PIOA_ODR;
  AT91_REG PIOA_OSR;
  AT91_REG Reserved7[1];
  AT91_REG PIOA_IFER;
  AT91_REG PIOA_IFDR;
  AT91_REG PIOA_IFSR;
  AT91_REG Reserved8[1];
  AT91_REG PIOA_SODR;
  AT91_REG PIOA_CODR;
  AT91_REG PIOA_ODSR;
  AT91_REG PIOA_PDSR;
  AT91_REG PIOA_IER;
  AT91_REG PIOA_IDR;
  AT91_REG PIOA_IMR;
  AT91_REG PIOA_ISR;
  AT91_REG PIOA_MDER;
  AT91_REG PIOA_MDDR;
  AT91_REG PIOA_MDSR;
  AT91_REG Reserved9[1];
  AT91_REG PIOA_PPUDR;
  AT91_REG PIOA_PPUER;
  AT91_REG PIOA_PPUSR;
  AT91_REG Reserved10[1];
  AT91_REG PIOA_ASR;
  AT91_REG PIOA_BSR;
  AT91_REG PIOA_ABSR;
  AT91_REG Reserved11[9];
  AT91_REG PIOA_OWER;
  AT91_REG PIOA_OWDR;
  AT91_REG PIOA_OWSR;
  AT91_REG Reserved12[469];
  AT91_REG PMC_SCER;
  AT91_REG PMC_SCDR;
  AT91_REG PMC_SCSR;
  AT91_REG Reserved13[1];
  AT91_REG PMC_PCER;
  AT91_REG PMC_PCDR;
  AT91_REG PMC_PCSR;
  AT91_REG Reserved14[1];
  AT91_REG PMC_MOR;
  AT91_REG PMC_MCFR;
  AT91_REG Reserved15[1];
  AT91_REG PMC_PLLR;
  AT91_REG PMC_MCKR;
  AT91_REG Reserved16[3];
  AT91_REG PMC_PCKR[3];
  AT91_REG Reserved17[5];
  AT91_REG PMC_IER;
  AT91_REG PMC_IDR;
  AT91_REG PMC_SR;
  AT91_REG PMC_IMR;
  AT91_REG Reserved18[36];
  AT91_REG RSTC_RCR;
  AT91_REG RSTC_RSR;
  AT91_REG RSTC_RMR;
  AT91_REG Reserved19[5];
  AT91_REG RTTC_RTMR;
  AT91_REG RTTC_RTAR;
  AT91_REG RTTC_RTVR;
  AT91_REG RTTC_RTSR;
  AT91_REG PITC_PIMR;
  AT91_REG PITC_PISR;
  AT91_REG PITC_PIVR;
  AT91_REG PITC_PIIR;
  AT91_REG WDTC_WDCR;
  AT91_REG WDTC_WDMR;
  AT91_REG WDTC_WDSR;
  AT91_REG Reserved20[5];
  AT91_REG VREG_MR;
} AT91S_SYS;
#line 171
#line 50
typedef struct _AT91S_SYS *
#line 171
AT91PS_SYS;
#line 198
#line 177
typedef struct _AT91S_AIC {
  AT91_REG AIC_SMR[32];
  AT91_REG AIC_SVR[32];
  AT91_REG AIC_IVR;
  AT91_REG AIC_FVR;
  AT91_REG AIC_ISR;
  AT91_REG AIC_IPR;
  AT91_REG AIC_IMR;
  AT91_REG AIC_CISR;
  AT91_REG Reserved0[2];
  AT91_REG AIC_IECR;
  AT91_REG AIC_IDCR;
  AT91_REG AIC_ICCR;
  AT91_REG AIC_ISCR;
  AT91_REG AIC_EOICR;
  AT91_REG AIC_SPU;
  AT91_REG AIC_DCR;
  AT91_REG Reserved1[1];
  AT91_REG AIC_FFER;
  AT91_REG AIC_FFDR;
  AT91_REG AIC_FFSR;
} AT91S_AIC;
#line 198
#line 177
typedef struct _AT91S_AIC *
#line 198
AT91PS_AIC;
#line 230
#line 219
typedef struct _AT91S_PDC {
  AT91_REG PDC_RPR;
  AT91_REG PDC_RCR;
  AT91_REG PDC_TPR;
  AT91_REG PDC_TCR;
  AT91_REG PDC_RNPR;
  AT91_REG PDC_RNCR;
  AT91_REG PDC_TNPR;
  AT91_REG PDC_TNCR;
  AT91_REG PDC_PTCR;
  AT91_REG PDC_PTSR;
} AT91S_PDC;
#line 230
#line 219
typedef struct _AT91S_PDC *
#line 230
AT91PS_PDC;
#line 267
#line 242
typedef struct _AT91S_DBGU {
  AT91_REG DBGU_CR;
  AT91_REG DBGU_MR;
  AT91_REG DBGU_IER;
  AT91_REG DBGU_IDR;
  AT91_REG DBGU_IMR;
  AT91_REG DBGU_CSR;
  AT91_REG DBGU_RHR;
  AT91_REG DBGU_THR;
  AT91_REG DBGU_BRGR;
  AT91_REG Reserved0[7];
  AT91_REG DBGU_CIDR;
  AT91_REG DBGU_EXID;
  AT91_REG DBGU_FNTR;
  AT91_REG Reserved1[45];
  AT91_REG DBGU_RPR;
  AT91_REG DBGU_RCR;
  AT91_REG DBGU_TPR;
  AT91_REG DBGU_TCR;
  AT91_REG DBGU_RNPR;
  AT91_REG DBGU_RNCR;
  AT91_REG DBGU_TNPR;
  AT91_REG DBGU_TNCR;
  AT91_REG DBGU_PTCR;
  AT91_REG DBGU_PTSR;
} AT91S_DBGU;
#line 267
#line 242
typedef struct _AT91S_DBGU *
#line 267
AT91PS_DBGU;
#line 348
#line 312
typedef struct _AT91S_PIO {
  AT91_REG PIO_PER;
  AT91_REG PIO_PDR;
  AT91_REG PIO_PSR;
  AT91_REG Reserved0[1];
  AT91_REG PIO_OER;
  AT91_REG PIO_ODR;
  AT91_REG PIO_OSR;
  AT91_REG Reserved1[1];
  AT91_REG PIO_IFER;
  AT91_REG PIO_IFDR;
  AT91_REG PIO_IFSR;
  AT91_REG Reserved2[1];
  AT91_REG PIO_SODR;
  AT91_REG PIO_CODR;
  AT91_REG PIO_ODSR;
  AT91_REG PIO_PDSR;
  AT91_REG PIO_IER;
  AT91_REG PIO_IDR;
  AT91_REG PIO_IMR;
  AT91_REG PIO_ISR;
  AT91_REG PIO_MDER;
  AT91_REG PIO_MDDR;
  AT91_REG PIO_MDSR;
  AT91_REG Reserved3[1];
  AT91_REG PIO_PPUDR;
  AT91_REG PIO_PPUER;
  AT91_REG PIO_PPUSR;
  AT91_REG Reserved4[1];
  AT91_REG PIO_ASR;
  AT91_REG PIO_BSR;
  AT91_REG PIO_ABSR;
  AT91_REG Reserved5[9];
  AT91_REG PIO_OWER;
  AT91_REG PIO_OWDR;
  AT91_REG PIO_OWSR;
} AT91S_PIO;
#line 348
#line 312
typedef struct _AT91S_PIO *
#line 348
AT91PS_PIO;










#line 354
typedef struct _AT91S_CKGR {
  AT91_REG CKGR_MOR;
  AT91_REG CKGR_MCFR;
  AT91_REG Reserved0[1];
  AT91_REG CKGR_PLLR;
} AT91S_CKGR;
#line 359
#line 354
typedef struct _AT91S_CKGR *




AT91PS_CKGR;
#line 408
#line 387
typedef struct _AT91S_PMC {
  AT91_REG PMC_SCER;
  AT91_REG PMC_SCDR;
  AT91_REG PMC_SCSR;
  AT91_REG Reserved0[1];
  AT91_REG PMC_PCER;
  AT91_REG PMC_PCDR;
  AT91_REG PMC_PCSR;
  AT91_REG Reserved1[1];
  AT91_REG PMC_MOR;
  AT91_REG PMC_MCFR;
  AT91_REG Reserved2[1];
  AT91_REG PMC_PLLR;
  AT91_REG PMC_MCKR;
  AT91_REG Reserved3[3];
  AT91_REG PMC_PCKR[3];
  AT91_REG Reserved4[5];
  AT91_REG PMC_IER;
  AT91_REG PMC_IDR;
  AT91_REG PMC_SR;
  AT91_REG PMC_IMR;
} AT91S_PMC;
#line 408
#line 387
typedef struct _AT91S_PMC *
#line 408
AT91PS_PMC;
#line 453
#line 449
typedef struct _AT91S_RSTC {
  AT91_REG RSTC_RCR;
  AT91_REG RSTC_RSR;
  AT91_REG RSTC_RMR;
} AT91S_RSTC;
#line 453
#line 449
typedef struct _AT91S_RSTC *



AT91PS_RSTC;
#line 486
#line 481
typedef struct _AT91S_RTTC {
  AT91_REG RTTC_RTMR;
  AT91_REG RTTC_RTAR;
  AT91_REG RTTC_RTVR;
  AT91_REG RTTC_RTSR;
} AT91S_RTTC;
#line 486
#line 481
typedef struct _AT91S_RTTC *




AT91PS_RTTC;
#line 509
#line 504
typedef struct _AT91S_PITC {
  AT91_REG PITC_PIMR;
  AT91_REG PITC_PISR;
  AT91_REG PITC_PIVR;
  AT91_REG PITC_PIIR;
} AT91S_PITC;
#line 509
#line 504
typedef struct _AT91S_PITC *




AT91PS_PITC;
#line 529
#line 525
typedef struct _AT91S_WDTC {
  AT91_REG WDTC_WDCR;
  AT91_REG WDTC_WDMR;
  AT91_REG WDTC_WDSR;
} AT91S_WDTC;
#line 529
#line 525
typedef struct _AT91S_WDTC *



AT91PS_WDTC;
#line 552
#line 550
typedef struct _AT91S_VREG {
  AT91_REG VREG_MR;
} AT91S_VREG;
#line 552
#line 550
typedef struct _AT91S_VREG *

AT91PS_VREG;
#line 568
#line 560
typedef struct _AT91S_MC {
  AT91_REG MC_RCR;
  AT91_REG MC_ASR;
  AT91_REG MC_AASR;
  AT91_REG Reserved0[21];
  AT91_REG MC_FMR;
  AT91_REG MC_FCR;
  AT91_REG MC_FSR;
} AT91S_MC;
#line 568
#line 560
typedef struct _AT91S_MC *







AT91PS_MC;
#line 662
#line 640
typedef struct _AT91S_SPI {
  AT91_REG SPI_CR;
  AT91_REG SPI_MR;
  AT91_REG SPI_RDR;
  AT91_REG SPI_TDR;
  AT91_REG SPI_SR;
  AT91_REG SPI_IER;
  AT91_REG SPI_IDR;
  AT91_REG SPI_IMR;
  AT91_REG Reserved0[4];
  AT91_REG SPI_CSR[4];
  AT91_REG Reserved1[48];
  AT91_REG SPI_RPR;
  AT91_REG SPI_RCR;
  AT91_REG SPI_TPR;
  AT91_REG SPI_TCR;
  AT91_REG SPI_RNPR;
  AT91_REG SPI_RNCR;
  AT91_REG SPI_TNPR;
  AT91_REG SPI_TNCR;
  AT91_REG SPI_PTCR;
  AT91_REG SPI_PTSR;
} AT91S_SPI;
#line 662
#line 640
typedef struct _AT91S_SPI *
#line 662
AT91PS_SPI;
#line 753
#line 722
typedef struct _AT91S_ADC {
  AT91_REG ADC_CR;
  AT91_REG ADC_MR;
  AT91_REG Reserved0[2];
  AT91_REG ADC_CHER;
  AT91_REG ADC_CHDR;
  AT91_REG ADC_CHSR;
  AT91_REG ADC_SR;
  AT91_REG ADC_LCDR;
  AT91_REG ADC_IER;
  AT91_REG ADC_IDR;
  AT91_REG ADC_IMR;
  AT91_REG ADC_CDR0;
  AT91_REG ADC_CDR1;
  AT91_REG ADC_CDR2;
  AT91_REG ADC_CDR3;
  AT91_REG ADC_CDR4;
  AT91_REG ADC_CDR5;
  AT91_REG ADC_CDR6;
  AT91_REG ADC_CDR7;
  AT91_REG Reserved1[44];
  AT91_REG ADC_RPR;
  AT91_REG ADC_RCR;
  AT91_REG ADC_TPR;
  AT91_REG ADC_TCR;
  AT91_REG ADC_RNPR;
  AT91_REG ADC_RNCR;
  AT91_REG ADC_TNPR;
  AT91_REG ADC_TNCR;
  AT91_REG ADC_PTCR;
  AT91_REG ADC_PTSR;
} AT91S_ADC;
#line 753
#line 722
typedef struct _AT91S_ADC *
#line 753
AT91PS_ADC;
#line 858
#line 829
typedef struct _AT91S_SSC {
  AT91_REG SSC_CR;
  AT91_REG SSC_CMR;
  AT91_REG Reserved0[2];
  AT91_REG SSC_RCMR;
  AT91_REG SSC_RFMR;
  AT91_REG SSC_TCMR;
  AT91_REG SSC_TFMR;
  AT91_REG SSC_RHR;
  AT91_REG SSC_THR;
  AT91_REG Reserved1[2];
  AT91_REG SSC_RSHR;
  AT91_REG SSC_TSHR;
  AT91_REG Reserved2[2];
  AT91_REG SSC_SR;
  AT91_REG SSC_IER;
  AT91_REG SSC_IDR;
  AT91_REG SSC_IMR;
  AT91_REG Reserved3[44];
  AT91_REG SSC_RPR;
  AT91_REG SSC_RCR;
  AT91_REG SSC_TPR;
  AT91_REG SSC_TCR;
  AT91_REG SSC_RNPR;
  AT91_REG SSC_RNCR;
  AT91_REG SSC_TNPR;
  AT91_REG SSC_TNCR;
  AT91_REG SSC_PTCR;
  AT91_REG SSC_PTSR;
} AT91S_SSC;
#line 858
#line 829
typedef struct _AT91S_SSC *
#line 858
AT91PS_SSC;
#line 954
#line 926
typedef struct _AT91S_USART {
  AT91_REG US_CR;
  AT91_REG US_MR;
  AT91_REG US_IER;
  AT91_REG US_IDR;
  AT91_REG US_IMR;
  AT91_REG US_CSR;
  AT91_REG US_RHR;
  AT91_REG US_THR;
  AT91_REG US_BRGR;
  AT91_REG US_RTOR;
  AT91_REG US_TTGR;
  AT91_REG Reserved0[5];
  AT91_REG US_FIDI;
  AT91_REG US_NER;
  AT91_REG Reserved1[1];
  AT91_REG US_IF;
  AT91_REG Reserved2[44];
  AT91_REG US_RPR;
  AT91_REG US_RCR;
  AT91_REG US_TPR;
  AT91_REG US_TCR;
  AT91_REG US_RNPR;
  AT91_REG US_RNCR;
  AT91_REG US_TNPR;
  AT91_REG US_TNCR;
  AT91_REG US_PTCR;
  AT91_REG US_PTSR;
} AT91S_USART;
#line 954
#line 926
typedef struct _AT91S_USART *
#line 954
AT91PS_USART;
#line 1034
#line 1021
typedef struct _AT91S_TWI {
  AT91_REG TWI_CR;
  AT91_REG TWI_MMR;
  AT91_REG Reserved0[1];
  AT91_REG TWI_IADR;
  AT91_REG TWI_CWGR;
  AT91_REG Reserved1[3];
  AT91_REG TWI_SR;
  AT91_REG TWI_IER;
  AT91_REG TWI_IDR;
  AT91_REG TWI_IMR;
  AT91_REG TWI_RHR;
  AT91_REG TWI_THR;
} AT91S_TWI;
#line 1034
#line 1021
typedef struct _AT91S_TWI *
#line 1034
AT91PS_TWI;
#line 1080
#line 1068
typedef struct _AT91S_TC {
  AT91_REG TC_CCR;
  AT91_REG TC_CMR;
  AT91_REG Reserved0[2];
  AT91_REG TC_CV;
  AT91_REG TC_RA;
  AT91_REG TC_RB;
  AT91_REG TC_RC;
  AT91_REG TC_SR;
  AT91_REG TC_IER;
  AT91_REG TC_IDR;
  AT91_REG TC_IMR;
} AT91S_TC;
#line 1080
#line 1068
typedef struct _AT91S_TC *
#line 1080
AT91PS_TC;
#line 1208
#line 1199
typedef struct _AT91S_TCB {
  AT91S_TC TCB_TC0;
  AT91_REG Reserved0[4];
  AT91S_TC TCB_TC1;
  AT91_REG Reserved1[4];
  AT91S_TC TCB_TC2;
  AT91_REG Reserved2[4];
  AT91_REG TCB_BCR;
  AT91_REG TCB_BMR;
} AT91S_TCB;
#line 1208
#line 1199
typedef struct _AT91S_TCB *








AT91PS_TCB;
#line 1239
#line 1232
typedef struct _AT91S_PWMC_CH {
  AT91_REG PWMC_CMR;
  AT91_REG PWMC_CDTYR;
  AT91_REG PWMC_CPRDR;
  AT91_REG PWMC_CCNTR;
  AT91_REG PWMC_CUPDR;
  AT91_REG PWMC_Reserved[3];
} AT91S_PWMC_CH;
#line 1239
#line 1232
typedef struct _AT91S_PWMC_CH *






AT91PS_PWMC_CH;
#line 1274
#line 1261
typedef struct _AT91S_PWMC {
  AT91_REG PWMC_MR;
  AT91_REG PWMC_ENA;
  AT91_REG PWMC_DIS;
  AT91_REG PWMC_SR;
  AT91_REG PWMC_IER;
  AT91_REG PWMC_IDR;
  AT91_REG PWMC_IMR;
  AT91_REG PWMC_ISR;
  AT91_REG Reserved0[55];
  AT91_REG PWMC_VR;
  AT91_REG Reserved1[64];
  AT91S_PWMC_CH PWMC_CH[32];
} AT91S_PWMC;
#line 1274
#line 1261
typedef struct _AT91S_PWMC *
#line 1274
AT91PS_PWMC;
#line 1319
#line 1302
typedef struct _AT91S_UDP {
  AT91_REG UDP_NUM;
  AT91_REG UDP_GLBSTATE;
  AT91_REG UDP_FADDR;
  AT91_REG Reserved0[1];
  AT91_REG UDP_IER;
  AT91_REG UDP_IDR;
  AT91_REG UDP_IMR;
  AT91_REG UDP_ISR;
  AT91_REG UDP_ICR;
  AT91_REG Reserved1[1];
  AT91_REG UDP_RSTEP;
  AT91_REG Reserved2[1];
  AT91_REG UDP_CSR[8];
  AT91_REG UDP_FDR[8];
  AT91_REG Reserved3[1];
  AT91_REG UDP_TXVC;
} AT91S_UDP;
#line 1319
#line 1302
typedef struct _AT91S_UDP *
#line 1319
AT91PS_UDP;
# 52 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/platforms/nxtmote/lib_AT91SAM7S256.h"
static __inline unsigned int AT91F_AIC_ConfigureIt(
AT91PS_AIC pAic, 
unsigned int irq_id, 
unsigned int priority, 
unsigned int src_type, 
void (*newHandler)(void ));
#line 81
static __inline void AT91F_AIC_EnableIt(
AT91PS_AIC pAic, 
unsigned int irq_id);
#line 1040
static __inline void AT91F_PMC_EnablePeriphClock(
AT91PS_PMC pPMC, 
unsigned int periphIds);
# 20 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/platforms/nxtmote/lib_extra_AT91SAM7S256.h"
static __inline unsigned int AT91F_AIC_ActiveID(
AT91PS_AIC pAic);
# 19 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/platforms/nxtmote/hardware.h"
typedef unsigned char UCHAR;
typedef unsigned short USHORT;
typedef unsigned char UBYTE;
typedef signed char SBYTE;
typedef unsigned short int UWORD;
typedef signed short int SWORD;
typedef unsigned long ULONG;
typedef signed long SLONG;

typedef ULONG *PULONG;
typedef USHORT *PUSHORT;
typedef UCHAR *PUCHAR;
typedef char *PSZ;
#line 50
extern void AT91F_Spurious_handler(void );



static __inline void __nesc_enable_interrupt();
#line 67
static __inline void __nesc_disable_interrupt();
#line 82
typedef uint32_t __nesc_atomic_t;

__inline __nesc_atomic_t __nesc_atomic_start(void )  ;
#line 100
__inline void __nesc_atomic_end(__nesc_atomic_t oldState)  ;
#line 129
const uint8_t TOSH_IRP_TABLE[33] = { 
0xFF, 
(unsigned int )0x7, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
(unsigned int )0x7, 
0xFF, 
0xFF, 
0x04, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF };





const uint8_t TOSH_IRQLEVEL_TABLE[33] = { 
0xFF, 
TRUE, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
FALSE, 
0xFF, 
0xFF, 
FALSE, 
FALSE, 
FALSE, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF };
#line 244
enum __nesc_unnamed4253 {

  SUCCESS_ = 0x0000, 
  INPROGRESS = 0x0001, 
  REQPIN = 0x0002, 
  NOMOREHANDLES = 0x8100, 
  NOSPACE = 0x8200, 
  NOMOREFILES = 0x8300, 
  EOFEXSPECTED = 0x8400, 
  ENDOFFILE = 0x8500, 
  NOTLINEARFILE = 0x8600, 
  FILENOTFOUND = 0x8700, 
  HANDLEALREADYCLOSED = 0x8800, 
  NOLINEARSPACE = 0x8900, 
  UNDEFINEDERROR = 0x8A00, 
  FILEISBUSY = 0x8B00, 
  NOWRITEBUFFERS = 0x8C00, 
  APPENDNOTPOSSIBLE = 0x8D00, 
  FILEISFULL = 0x8E00, 
  FILEEXISTS = 0x8F00, 
  MODULENOTFOUND = 0x9000, 
  OUTOFBOUNDERY = 0x9100, 
  ILLEGALFILENAME = 0x9200, 
  ILLEGALHANDLE = 0x9300, 
  BTBUSY = 0x9400, 
  BTCONNECTFAIL = 0x9500, 
  BTTIMEOUT = 0x9600, 
  FILETX_TIMEOUT = 0x9700, 
  FILETX_DSTEXISTS = 0x9800, 
  FILETX_SRCMISSING = 0x9900, 
  FILETX_STREAMERROR = 0x9A00, 
  FILETX_CLOSEERROR = 0x9B00, 
  BTWAIT = 0x9C00
};


enum __nesc_unnamed4254 {

  OPENREAD = 0x80, 
  OPENWRITE = 0x81, 
  READ = 0x82, 
  WRITE = 0x83, 
  CLOSE = 0x84, 
  DELETE = 0x85, 
  FINDFIRST = 0x86, 
  FINDNEXT = 0x87, 
  VERSIONS = 0x88, 
  OPENWRITELINEAR = 0x89, 
  OPENREADLINEAR = 0x8A, 
  OPENWRITEDATA = 0x8B, 
  OPENAPPENDDATA = 0x8C, 
  FINDFIRSTMODULE = 0x90, 
  FINDNEXTMODULE = 0x91, 
  CLOSEMODHANDLE = 0x92, 
  IOMAPREAD = 0x94, 
  IOMAPWRITE = 0x95, 
  BOOTCMD = 0x97, 
  SETBRICKNAME = 0x98, 
  BTGETADR = 0x9A, 
  DEVICEINFO = 0x9B, 
  DELETEUSERFLASH = 0xA0, 
  POLLCMDLEN = 0xA1, 
  POLLCMD = 0xA2, 
  RENAMEFILE = 0xA3, 
  BTFACTORYRESET = 0xA4
};





enum __nesc_unnamed4255 {

  BT_STATE_VISIBLE = 0x01, 
  BT_STATE_CONNECTED = 0x02, 
  BT_STATE_OFF = 0x04, 
  BT_ERROR_ATTENTION = 0x08, 
  BT_CONNECT_REQUEST = 0x40, 
  BT_PIN_REQUEST = 0x80
};
#line 358
enum __nesc_unnamed4256 {

  RC_START_PROGRAM, 
  RC_STOP_PROGRAM, 
  RC_PLAY_SOUND_FILE, 
  RC_PLAY_TONE, 
  RC_SET_OUT_STATE, 
  RC_SET_IN_MODE, 
  RC_GET_OUT_STATE, 
  RC_GET_IN_VALS, 
  RC_RESET_IN_VAL, 
  RC_MESSAGE_WRITE, 
  RC_RESET_POSITION, 
  RC_GET_BATT_LVL, 
  RC_STOP_SOUND, 
  RC_KEEP_ALIVE, 
  RC_LS_GET_STATUS, 
  RC_LS_WRITE, 
  RC_LS_READ, 
  RC_GET_CURR_PROGRAM, 
  RC_GET_BUTTON_STATE, 
  RC_MESSAGE_READ, 
  NUM_RC_OPCODES
};


enum __nesc_unnamed4257 {

  POWERDOWN = 0x5A00, 
  BOOT = 0xA55A
};



enum __nesc_unnamed4258 {

  UI_UPDATE = 0x01, 
  UI_DISABLE_LEFT_RIGHT_ENTER = 0x02, 
  UI_DISABLE_EXIT = 0x04, 
  UI_REDRAW_STATUS = 0x08, 
  UI_RESET_SLEEP_TIMER = 0x10, 
  UI_EXECUTE_LMS_FILE = 0x20, 
  UI_BUSY = 0x40, 
  UI_ENABLE_STATUS_UPDATE = 0x80
};



enum __nesc_unnamed4259 {

  DISPLAY_ON = 0x01, 
  DISPLAY_REFRESH = 0x02, 
  DISPLAY_POPUP = 0x08, 
  DISPLAY_REFRESH_DISABLED = 0x40, 
  DISPLAY_BUSY = 0x80
};


enum __nesc_unnamed4260 {

  MSG_BEGIN_INQUIRY, 
  MSG_CANCEL_INQUIRY, 
  MSG_CONNECT, 
  MSG_OPEN_PORT, 
  MSG_LOOKUP_NAME, 
  MSG_ADD_DEVICE, 
  MSG_REMOVE_DEVICE, 
  MSG_DUMP_LIST, 
  MSG_CLOSE_CONNECTION, 
  MSG_ACCEPT_CONNECTION, 
  MSG_PIN_CODE, 
  MSG_OPEN_STREAM, 
  MSG_START_HEART, 
  MSG_HEARTBEAT, 
  MSG_INQUIRY_RUNNING, 
  MSG_INQUIRY_RESULT, 
  MSG_INQUIRY_STOPPED, 
  MSG_LOOKUP_NAME_RESULT, 
  MSG_LOOKUP_NAME_FAILURE, 
  MSG_CONNECT_RESULT, 
  MSG_RESET_INDICATION, 
  MSG_REQUEST_PIN_CODE, 
  MSG_REQUEST_CONNECTION, 
  MSG_LIST_RESULT, 
  MSG_LIST_ITEM, 
  MSG_LIST_DUMP_STOPPED, 
  MSG_CLOSE_CONNECTION_RESULT, 
  MSG_PORT_OPEN_RESULT, 
  MSG_SET_DISCOVERABLE, 
  MSG_CLOSE_PORT, 
  MSG_CLOSE_PORT_RESULT, 
  MSG_PIN_CODE_ACK, 
  MSG_DISCOVERABLE_ACK, 
  MSG_SET_FRIENDLY_NAME, 
  MSG_SET_FRIENDLY_NAME_ACK, 
  MSG_GET_LINK_QUALITY, 
  MSG_LINK_QUALITY_RESULT, 
  MSG_SET_FACTORY_SETTINGS, 
  MSG_SET_FACTORY_SETTINGS_ACK, 
  MSG_GET_LOCAL_ADDR, 
  MSG_GET_LOCAL_ADDR_RESULT, 
  MSG_GET_FRIENDLY_NAME, 
  MSG_GET_DISCOVERABLE, 
  MSG_GET_PORT_OPEN, 
  MSG_GET_FRIENDLY_NAME_RESULT, 
  MSG_GET_DISCOVERABLE_RESULT, 
  MSG_GET_PORT_OPEN_RESULT, 
  MSG_GET_VERSION, 
  MSG_GET_VERSION_RESULT, 
  MSG_GET_BRICK_STATUSBYTE_RESULT, 
  MSG_SET_BRICK_STATUSBYTE_RESULT, 
  MSG_GET_BRICK_STATUSBYTE, 
  MSG_SET_BRICK_STATUSBYTE
};
# 9 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/platforms/nxtmote/nxt.h"
enum __nesc_unnamed4261 {

  NOS_OF_AVR_OUTPUTS = 4, 
  NOS_OF_AVR_BTNS = 4, 
  NOS_OF_AVR_INPUTS = 4
};






#line 16
typedef struct __nesc_unnamed4262 {

  UWORD AdValue[NOS_OF_AVR_INPUTS];
  UWORD Buttons;
  UWORD Battery;
} IOFROMAVR;








#line 23
typedef struct __nesc_unnamed4263 {

  UBYTE Power;
  UBYTE PwmFreq;
  SBYTE PwmValue[NOS_OF_AVR_OUTPUTS];
  UBYTE OutputMode;
  UBYTE InputPower;
} IOTOAVR;
# 62 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
#line 62
static error_t PlatformP__InitL0__default__init(void );
#line 62
static error_t PlatformP__InitL3__default__init(void );
#line 62
static error_t PlatformP__PInit__default__init(void );
#line 62
static error_t PlatformP__InitL1__default__init(void );
# 29 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/HplAT91Interrupt.nc"
static void HplAT91InterruptM__AT91Irq__default__fired(
# 10 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/HplAT91InterruptM.nc"
uint8_t arg_0x104612df0);
# 19 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/HplAT91Interrupt.nc"
static void HplAT91InterruptM__AT91Irq__enable(
# 10 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/HplAT91InterruptM.nc"
uint8_t arg_0x104612df0);
# 14 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/HplAT91Interrupt.nc"
static error_t HplAT91InterruptM__AT91Irq__allocate(
# 10 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/HplAT91InterruptM.nc"
uint8_t arg_0x104612df0);
# 62 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Init.nc"
static error_t HplAT91PitM__Init__init(void );
# 75 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/TaskBasic.nc"
static void HplAT91PitM__fireFromTask__runTask(void );
# 29 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/HplAT91Interrupt.nc"
static void HplAT91PitM__SysIrq__fired(void );
# 22 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/timer/HplAT91Pit.nc"
static void HplAT91PitM__HplAT91Pit__default__fired(void );









static void HplAT91PitM__HplAT91Pit__default__firedTask(uint32_t taskMiss);
# 62 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__default__init(void );
# 67 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 56 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x1045c5060);
# 75 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 56 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x1045c5060);
# 57 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
#line 72
static void SchedulerBasicP__Scheduler__taskLoop(void );
#line 65
static bool SchedulerBasicP__Scheduler__runNextTask(void );
# 76 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 60 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Boot.nc"
static void NullC__Boot__booted(void );
# 62 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Init.nc"
static error_t PlatformP__InitL2__init(void );
#line 62
static error_t PlatformP__InitL0__init(void );
#line 62
static error_t PlatformP__InitL3__init(void );
#line 62
static error_t PlatformP__PInit__init(void );
#line 62
static error_t PlatformP__InitL1__init(void );
# 51 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/platforms/nxtmote/PlatformP.nc"
static inline error_t PlatformP__Init__init(void );
#line 133
static inline error_t PlatformP__InitL0__default__init(void );
static inline error_t PlatformP__InitL1__default__init(void );

static inline error_t PlatformP__InitL3__default__init(void );
static inline error_t PlatformP__PInit__default__init(void );
# 29 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/HplAT91Interrupt.nc"
static void HplAT91InterruptM__AT91Irq__fired(
# 10 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/HplAT91InterruptM.nc"
uint8_t arg_0x104612df0);
#line 22
void irqhandler(void )   ;









void fiqhandler(void ) __attribute((interrupt("FIQ")))   ;




static inline void HplAT91InterruptM__enable(uint8_t id);
#line 51
static inline error_t HplAT91InterruptM__allocate(uint8_t id, bool level, uint8_t priority);
#line 79
static inline error_t HplAT91InterruptM__AT91Irq__allocate(uint8_t id);




static inline void HplAT91InterruptM__AT91Irq__enable(uint8_t id);
#line 113
static inline void HplAT91InterruptM__AT91Irq__default__fired(uint8_t id);
# 67 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/TaskBasic.nc"
static error_t HplAT91PitM__fireFromTask__postTask(void );
# 19 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/HplAT91Interrupt.nc"
static void HplAT91PitM__SysIrq__enable(void );
#line 14
static error_t HplAT91PitM__SysIrq__allocate(void );
# 22 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/timer/HplAT91Pit.nc"
static void HplAT91PitM__HplAT91Pit__fired(void );









static void HplAT91PitM__HplAT91Pit__firedTask(uint32_t taskMiss);
# 21 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/timer/HplAT91PitM.nc"
enum HplAT91PitM____nesc_unnamed4264 {
#line 21
  HplAT91PitM__fireFromTask = 0U
};
#line 21
typedef int HplAT91PitM____nesc_sillytask_fireFromTask[HplAT91PitM__fireFromTask];
#line 19
uint32_t HplAT91PitM__taskMiss;



static inline error_t HplAT91PitM__Init__init(void );
#line 58
static inline void HplAT91PitM__SysIrq__fired(void );
#line 80
static void HplAT91PitM__fireFromTask__runTask(void );







static inline void HplAT91PitM__HplAT91Pit__default__firedTask(uint32_t tM);


static inline void HplAT91PitM__HplAT91Pit__default__fired(void );
# 62 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__init(void );
# 60 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Boot.nc"
static void RealMainP__Boot__booted(void );
# 62 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Init.nc"
static error_t RealMainP__PlatformInit__init(void );
# 57 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Scheduler.nc"
static void RealMainP__Scheduler__init(void );
#line 72
static void RealMainP__Scheduler__taskLoop(void );
#line 65
static bool RealMainP__Scheduler__runNextTask(void );
# 63 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/system/RealMainP.nc"
int main(void )   ;
#line 105
static inline error_t RealMainP__SoftwareInit__default__init(void );
# 75 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
# 56 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x1045c5060);
# 76 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 61 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4265 {

  SchedulerBasicP__NUM_TASKS = 1U, 
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
static inline error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id);




static inline void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id);
# 22 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/platforms/nxtmote/McuSleepC.nc"
static inline void McuSleepC__McuSleep__sleep(void );
# 58 "NullC.nc"
static inline void NullC__Boot__booted(void );
# 84 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/platforms/nxtmote/hardware.h"
__inline  __nesc_atomic_t __nesc_atomic_start(void )
#line 84
{
  uint32_t result = 0;











  return result;
}

__inline  void __nesc_atomic_end(__nesc_atomic_t oldState)
#line 100
{


  oldState &= 0x000000C0;










  return;
}

# 20 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/platforms/nxtmote/lib_extra_AT91SAM7S256.h"
static __inline unsigned int AT91F_AIC_ActiveID(
AT91PS_AIC pAic)
{
  return pAic->AIC_ISR;
}

# 91 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/timer/HplAT91PitM.nc"
static inline void HplAT91PitM__HplAT91Pit__default__fired(void )
#line 91
{
}

# 22 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/timer/HplAT91Pit.nc"
inline static void HplAT91PitM__HplAT91Pit__fired(void ){
#line 22
  HplAT91PitM__HplAT91Pit__default__fired();
#line 22
}
#line 22
# 97 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/system/SchedulerBasicP.nc"
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

#line 170
static inline error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id)
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

# 67 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/TaskBasic.nc"
inline static error_t HplAT91PitM__fireFromTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(HplAT91PitM__fireFromTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 58 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/timer/HplAT91PitM.nc"
static inline void HplAT91PitM__SysIrq__fired(void )
{
  uint32_t pitregtmp;

  error_t taskRes;


  if (* (AT91_REG *)0xFFFFFD34 & ((unsigned int )0x1 << 0)) {
      pitregtmp = * (AT91_REG *)0xFFFFFD38;


      taskRes = HplAT91PitM__fireFromTask__postTask();
      if (taskRes != SUCCESS) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 71
            {
#line 71
              HplAT91PitM__taskMiss++;
            }
#line 72
            __nesc_atomic_end(__nesc_atomic); }
        }

      HplAT91PitM__HplAT91Pit__fired();
    }
}

# 113 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/HplAT91InterruptM.nc"
static inline void HplAT91InterruptM__AT91Irq__default__fired(uint8_t id)
{
  return;
}

# 29 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/HplAT91Interrupt.nc"
inline static void HplAT91InterruptM__AT91Irq__fired(uint8_t arg_0x104612df0){
#line 29
  switch (arg_0x104612df0) {
#line 29
    case (unsigned int )1:
#line 29
      HplAT91PitM__SysIrq__fired();
#line 29
      break;
#line 29
    default:
#line 29
      HplAT91InterruptM__AT91Irq__default__fired(arg_0x104612df0);
#line 29
      break;
#line 29
    }
#line 29
}
#line 29
# 124 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__Scheduler__init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

# 57 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__init(void ){
#line 57
  SchedulerBasicP__Scheduler__init();
#line 57
}
#line 57
# 137 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/platforms/nxtmote/PlatformP.nc"
static inline error_t PlatformP__PInit__default__init(void )
#line 137
{
#line 137
  return SUCCESS;
}

# 62 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Init.nc"
inline static error_t PlatformP__PInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = PlatformP__PInit__default__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 136 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/platforms/nxtmote/PlatformP.nc"
static inline error_t PlatformP__InitL3__default__init(void )
#line 136
{
#line 136
  return SUCCESS;
}

# 62 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Init.nc"
inline static error_t PlatformP__InitL3__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = PlatformP__InitL3__default__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 81 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/platforms/nxtmote/lib_AT91SAM7S256.h"
static __inline void AT91F_AIC_EnableIt(
AT91PS_AIC pAic, 
unsigned int irq_id)
{

  pAic->AIC_IECR = 0x1 << irq_id;
}

#line 1040
static __inline void AT91F_PMC_EnablePeriphClock(
AT91PS_PMC pPMC, 
unsigned int periphIds)
{
  pPMC->PMC_PCER = periphIds;
}

# 37 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/HplAT91InterruptM.nc"
static inline void HplAT91InterruptM__enable(uint8_t id)
{
  /* atomic removed: atomic calls only */
#line 39
  {
    if (id < 34) {
        ((AT91PS_TC )0xFFFA0000)->TC_IER = (unsigned int )0x1 << 4;
        AT91F_PMC_EnablePeriphClock((AT91PS_PMC )0xFFFFFC00, id);

        AT91F_AIC_EnableIt((AT91PS_AIC )0xFFFFF000, id);
      }
  }

  return;
}

#line 84
static inline void HplAT91InterruptM__AT91Irq__enable(uint8_t id)
{
  HplAT91InterruptM__enable(id);
  return;
}

# 19 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/HplAT91Interrupt.nc"
inline static void HplAT91PitM__SysIrq__enable(void ){
#line 19
  HplAT91InterruptM__AT91Irq__enable((unsigned int )1);
#line 19
}
#line 19
# 52 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/platforms/nxtmote/lib_AT91SAM7S256.h"
static __inline unsigned int AT91F_AIC_ConfigureIt(
AT91PS_AIC pAic, 
unsigned int irq_id, 
unsigned int priority, 
unsigned int src_type, 
void (*newHandler)(void ))
{
  unsigned int oldHandler;
  unsigned int mask;

  oldHandler = pAic->AIC_SVR[irq_id];

  mask = 0x1 << irq_id;

  pAic->AIC_IDCR = mask;

  pAic->AIC_SVR[irq_id] = (unsigned int )newHandler;

  pAic->AIC_SMR[irq_id] = src_type | priority;

  pAic->AIC_ICCR = mask;

  return oldHandler;
}

# 51 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/HplAT91InterruptM.nc"
static inline error_t HplAT91InterruptM__allocate(uint8_t id, bool level, uint8_t priority)
{
  uint32_t srctype;

#line 54
  if (level) {
      srctype = (unsigned int )0x0 << 5;
    }
  else 
    {
      srctype = (unsigned int )0x1 << 5;
    }


  AT91F_AIC_ConfigureIt((AT91PS_AIC )0xFFFFF000, id, priority, srctype, irqhandler);

  return TRUE;
}

#line 79
static inline error_t HplAT91InterruptM__AT91Irq__allocate(uint8_t id)
{
  return HplAT91InterruptM__allocate(id, TOSH_IRQLEVEL_TABLE[id], TOSH_IRP_TABLE[id]);
}

# 14 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/HplAT91Interrupt.nc"
inline static error_t HplAT91PitM__SysIrq__allocate(void ){
#line 14
  unsigned char __nesc_result;
#line 14

#line 14
  __nesc_result = HplAT91InterruptM__AT91Irq__allocate((unsigned int )1);
#line 14

#line 14
  return __nesc_result;
#line 14
}
#line 14
# 23 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/timer/HplAT91PitM.nc"
static inline error_t HplAT91PitM__Init__init(void )
{





  HplAT91PitM__taskMiss = 0;

  HplAT91PitM__SysIrq__allocate();
  HplAT91PitM__SysIrq__enable();

  return SUCCESS;
}

# 62 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Init.nc"
inline static error_t PlatformP__InitL2__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = HplAT91PitM__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 134 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/platforms/nxtmote/PlatformP.nc"
static inline error_t PlatformP__InitL1__default__init(void )
#line 134
{
#line 134
  return SUCCESS;
}

# 62 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Init.nc"
inline static error_t PlatformP__InitL1__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = PlatformP__InitL1__default__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 133 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/platforms/nxtmote/PlatformP.nc"
static inline error_t PlatformP__InitL0__default__init(void )
#line 133
{
#line 133
  return SUCCESS;
}

# 62 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Init.nc"
inline static error_t PlatformP__InitL0__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = PlatformP__InitL0__default__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 51 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/platforms/nxtmote/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
#line 51
{



  uint32_t TmpReset;

  AT91PS_PMC pPMC = (AT91PS_PMC )0xFFFFFC00;

  (


  (AT91PS_MC )0xFFFFFF00)->MC_FMR = (((unsigned int )0xFF << 16) & (72 << 16)) | ((unsigned int )0x1 << 8);
  (

  (AT91PS_WDTC )0xFFFFFD40)->WDTC_WDMR = (unsigned int )0x1 << 15;





  pPMC->PMC_MOR = (((unsigned int )0xFF << 8) & (0x06 << 8)) | ((unsigned int )0x1 << 0);


  while (!(pPMC->PMC_SR & ((unsigned int )0x1 << 0))) ;









  pPMC->PMC_PLLR = ((((unsigned int )0xFF << 0) & 14) | ((
  (unsigned int )0x3F << 8) & (28 << 8))) | ((
  (unsigned int )0x7FF << 16) & (72 << 16));


  while (!(pPMC->PMC_SR & ((unsigned int )0x1 << 2))) ;
  while (!(pPMC->PMC_SR & ((unsigned int )0x1 << 3))) ;



  pPMC->PMC_MCKR = (unsigned int )0x1 << 2;
  while (!(pPMC->PMC_SR & ((unsigned int )0x1 << 3))) ;

  pPMC->PMC_MCKR |= (unsigned int )0x3;
  while (!(pPMC->PMC_SR & ((unsigned int )0x1 << 3))) ;







  pPMC->PMC_PCER = 1 << (unsigned int )2;
  (



  (AT91PS_AIC )0xFFFFF000)->AIC_SPU = (int )AT91F_Spurious_handler;

  * (AT91_REG *)0xFFFFFD08 = 0xA5000401;
  * (AT91_REG *)0xFFFFF138 = 1;



  * (AT91_REG *)0xFFFFFD30 = (48054850L / 16 / 1000 | ((unsigned int )0x1 << 24)) | ((unsigned int )0x1 << 25);
  TmpReset = * (AT91_REG *)0xFFFFFD38;
  TmpReset = TmpReset;

  * (AT91_REG *)0xFFFFFC10 = 1L << (unsigned int )9;

  PlatformP__InitL0__init();
  PlatformP__InitL1__init();
  PlatformP__InitL2__init();
  PlatformP__InitL3__init();
  PlatformP__PInit__init();

  return SUCCESS;
}

# 62 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Init.nc"
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
# 65 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Scheduler.nc"
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
# 88 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/timer/HplAT91PitM.nc"
static inline void HplAT91PitM__HplAT91Pit__default__firedTask(uint32_t tM)
#line 88
{
}

# 32 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/timer/HplAT91Pit.nc"
inline static void HplAT91PitM__HplAT91Pit__firedTask(uint32_t taskMiss){
#line 32
  HplAT91PitM__HplAT91Pit__default__firedTask(taskMiss);
#line 32
}
#line 32
# 105 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/system/RealMainP.nc"
static inline error_t RealMainP__SoftwareInit__default__init(void )
#line 105
{
#line 105
  return SUCCESS;
}

# 62 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = RealMainP__SoftwareInit__default__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 54 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/platforms/nxtmote/hardware.h"
static __inline void __nesc_enable_interrupt()
#line 54
{
  uint32_t statusReg = 0;

   __asm volatile (
  "mrs %0,CPSR\n\t"
  "bic %0,%1,#0xc0\n\t"
  "msr CPSR_c, %1" : 
  "=r"(statusReg) : 
  "0"(statusReg));

  return;
}

# 58 "NullC.nc"
static inline void NullC__Boot__booted(void )
#line 58
{
}

# 60 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 60
  NullC__Boot__booted();
#line 60
}
#line 60
# 175 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id)
{
}

# 75 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/TaskBasic.nc"
inline static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x1045c5060){
#line 75
  switch (arg_0x1045c5060) {
#line 75
    case HplAT91PitM__fireFromTask:
#line 75
      HplAT91PitM__fireFromTask__runTask();
#line 75
      break;
#line 75
    default:
#line 75
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x1045c5060);
#line 75
      break;
#line 75
    }
#line 75
}
#line 75
# 22 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/platforms/nxtmote/McuSleepC.nc"
static inline void McuSleepC__McuSleep__sleep(void )
#line 22
{
}

# 76 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP__McuSleep__sleep(void ){
#line 76
  McuSleepC__McuSleep__sleep();
#line 76
}
#line 76
# 78 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/system/SchedulerBasicP.nc"
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

# 72 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__taskLoop(void ){
#line 72
  SchedulerBasicP__Scheduler__taskLoop();
#line 72
}
#line 72
# 67 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/platforms/nxtmote/hardware.h"
static __inline void __nesc_disable_interrupt()
#line 67
{
#line 79
  return;
}

# 22 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/HplAT91InterruptM.nc"
  void irqhandler(void )
#line 22
{
  uint32_t irqID;

  irqID = AT91F_AIC_ActiveID((AT91PS_AIC )0xFFFFF000);

  HplAT91InterruptM__AT91Irq__fired(irqID);

  return;
}

__attribute((interrupt("FIQ")))   void fiqhandler(void )
#line 32
{
  return;
}

# 63 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/system/RealMainP.nc"
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

# 134 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/system/SchedulerBasicP.nc"
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

# 80 "/Users/Xavier/Documents/tinyOS/trunk/tinyOSsourceNXT/tos/chips/at91/timer/HplAT91PitM.nc"
static void HplAT91PitM__fireFromTask__runTask(void )
#line 80
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 82
    {
      HplAT91PitM__HplAT91Pit__firedTask(HplAT91PitM__taskMiss);
      HplAT91PitM__taskMiss = 0;
    }
#line 85
    __nesc_atomic_end(__nesc_atomic); }
}

