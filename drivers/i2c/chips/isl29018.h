#ifndef _ISL29018_H_
#define _ISL29018_H_

#define ISL29018_DRIVER_NAME "isl29018"

//. Operation mode
#define ISL29018_OFF_MODE               0
#define ISL29018_ALS_ONCE_MODE          1       //. Ambinet light sensor
#define ISL29018_IR_ONCE_MODE           2
#define ISL29018_PROX_ONCE_MODE         3       //. Proximity
#define ISL29018_RESERVED_MODE          4
#define ISL29018_ALS_CONTINUE_MODE      5
#define ISL29018_IR_CONTINUE_MODE       6
#define ISL29018_PROX_CONTINUE_MODE     7

//. Interrupt Persist
#define ISL29018_INT_PERSIST_CYCLE_1    0	//. Number of integration cycles
#define ISL29018_INT_PERSIST_CYCLE_4    1
#define ISL29018_INT_PERSIST_CYCLE_8    2
#define ISL29018_INT_PERSIST_CYCLE_16   3

//. Proximity Sensing Scheme
#define ISL29018_Scheme_0               0       //. Sensing IR From LED and ambient
#define ISL29018_Scheme_1               1       //. Sensing IR from LED with ambient IR rejection

//. Modulation Frequence
#define ISL29018_Modu_Freq_DC           0
#define ISL29018_Modu_Freq_360          1       //. save power

//. Current source Capability at IRDR Pin
#define ISL29018_IRDR_CURRENT_12_5      0       //. 12.5 mA IR LED driver
#define ISL29018_IRDR_CURRENT_25        1       //. 25   mA
#define ISL29018_IRDR_CURRENT_50        2       //. 50   mA
#define ISL29018_IRDR_CURRENT_100       3       //. 100  mA

//. RESOLUTION(n-BIT ADC)
#define ISL29018_RESOLUTION_16          0       //. 2^16 = 65536
#define ISL29018_RESOLUTION_12          1       //. 2^12 = 4096
#define ISL29018_RESOLUTION_8           2       //. 2^8  = 256
#define ISL29018_RESOLUTION_4           3       //. 2^4  = 16

//. Range/FSR LUX
#define ISL29018_RANGE_1                0       //. max: 1,000
#define ISL29018_RANGE_2                1       //. max: 4,000
#define ISL29018_RANGE_3                2       //. max: 16,000
#define ISL29018_RANGE_4                3       //. max: 64,000

#define ISL29018_IOC_MAGIC              'I'
#define ISL29018_SET_MODE               _IOW(ISL29018_IOC_MAGIC,  1, unsigned char)
#define ISL29018_GET_MODE               _IOW(ISL29018_IOC_MAGIC,  2, unsigned char)
#define ISL29018_GET_INT_FLAG           _IOW(ISL29018_IOC_MAGIC,  3, unsigned char)     //. output: '1' means interrupt triggered
#define ISL29018_SET_INT_PERSIST_CYCLE  _IOW(ISL29018_IOC_MAGIC,  4, unsigned char)     //. interrup presist: interrupt triggered after setting cycles
#define ISL29018_GET_INT_PERSIST_CYCLE  _IOW(ISL29018_IOC_MAGIC,  5, unsigned char)
#define ISL29018_SET_PROX_SCHEME        _IOW(ISL29018_IOC_MAGIC,  6, unsigned char)
#define ISL29018_GET_PROX_SCHEME        _IOW(ISL29018_IOC_MAGIC,  7, unsigned char)
#define ISL29018_SET_MODULATION_FREQ    _IOW(ISL29018_IOC_MAGIC,  8, unsigned char)
#define ISL29018_GET_MODULATION_FREQ    _IOW(ISL29018_IOC_MAGIC,  9, unsigned char)
#define ISL29018_SET_IRDR_CURRENT       _IOW(ISL29018_IOC_MAGIC, 10, unsigned char)
#define ISL29018_GET_IRDR_CURRENT       _IOW(ISL29018_IOC_MAGIC, 11, unsigned char)
#define ISL29018_SET_RESOLUTION         _IOW(ISL29018_IOC_MAGIC, 12, unsigned char)
#define ISL29018_GET_RESOLUTION         _IOW(ISL29018_IOC_MAGIC, 13, unsigned char)
#define ISL29018_SET_RANGE              _IOW(ISL29018_IOC_MAGIC, 14, unsigned char)
#define ISL29018_GET_RANGE              _IOW(ISL29018_IOC_MAGIC, 15, unsigned char)
#define ISL29018_GET_DATA               _IOW(ISL29018_IOC_MAGIC, 16, short)
#define ISL29018_SET_INT_LOW_THRES      _IOW(ISL29018_IOC_MAGIC, 17, short)
#define ISL29018_GET_INT_LOW_THRES      _IOW(ISL29018_IOC_MAGIC, 18, short)
#define ISL29018_SET_INT_HIGH_THRES     _IOW(ISL29018_IOC_MAGIC, 19, short)
#define ISL29018_GET_INT_HIGH_THRES     _IOW(ISL29018_IOC_MAGIC, 20, short)

#define ISL29018_IOC_MAXNR              21

#endif  // _ISL29018_H_

