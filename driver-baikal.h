#ifndef __DEVICE_BAIKAL_H__
#define __DEVICE_BAIKAL_H__

#include "miner.h"

#define BAIKAL_MAXMINERS    (8)
#define BAIKAL_MAXASICS        (20)
#define BAIKAL_WORK_FIFO        (48)

#define BAIKAL_CLK_MIN  (150)
#define BAIKAL_CLK_DEF  (300)
#define BAIKAL_CLK_MAX    (390)
#define BAIKAL_CUTOFF_TEMP      (70)
#define BAIKAL_FANSPEED_DEF     (100)
#define BAIKAL_FANSPEED_MAX     (100)
#define BAIKAL_RECOVER_TEMP     (50)

#define BAIKAL_RESET    (0x01)
#define BAIKAL_GET_INFO        (0x02)
#define BAIKAL_SET_OPTION    (0x03)
#define BAIKAL_SEND_WORK    (0x04)
#define BAIKAL_GET_RESULT    (0x05)
#define BAIKAL_SET_ID    (0x06)
#define BAIKAL_SET_IDLE    (0x07)

#define BAIKAL_MINER_TYPE_NONE  (0x00)
#define BAIKAL_MINER_TYPE_MINI  (0x01)
#define BAIKAL_MINER_TYPE_CUBE  (0x02)

struct asic_info {
    uint32_t nonce;
    uint32_t error;
};

struct miner_info {
    int     thr_id;
    int     asic_count;  
    int     asic_count_r;  
    int     unit_count;
	int		temp;  
    int     clock;
    int     bbg;
    bool    working;
    bool    overheated;
    uint8_t fw_ver;
    uint8_t hw_ver;
    uint8_t asic_ver;    
    uint32_t nonce;
    uint32_t error;    
    double working_diff;    
    struct asic_info asics[BAIKAL_MAXASICS]; 
    uint8_t work_idx;
    struct work *works[BAIKAL_WORK_FIFO];
    cgtimer_t start_time;
};


struct baikal_info {
    struct pool pool;
    int miner_count;
    int clock;
    uint8_t cutofftemp;
    uint8_t fanspeed;		// percent
    uint8_t recovertemp;
	pthread_t *process_thr;
    struct miner_info miners[BAIKAL_MAXMINERS];    
    uint8_t miner_type;
};

typedef struct {
    uint8_t     miner_id;
    uint8_t     cmd;
    uint8_t     param;
    uint8_t     dest;
    uint8_t     data[256];
    uint32_t    len;
} baikal_msg;


#endif /* __DEVICE_BAIKAL_H__ */

