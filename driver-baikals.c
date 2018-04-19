/*

 * Copyright 2012-2013 Andrew Smith
 * Copyright 2012 Luke Dashjr
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */
#include "config.h"
#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>

#include "logging.h"
#include "miner.h"
#include "usbutils.h"
#include "util.h"
#include "config_parser.h"
#include "driver-baikal.h"
#include "compat.h"
#include "algorithm.h"

#define BAIKAL_IO_PORT      "/dev/ttyS2"
#define BAIKAL_IO_SPEED     (B115200)
#define BAIKAL_GPIO_RESET   "/sys/class/gpio_sw/PA21/data"      // orange pi one
#define BAIKAL_GPIO_EXIST   "/sys/class/gpio_sw/PA8/data"

#define BAIKAL_GPIO_RESET_CUBE  "/sys/class/gpio_sw/PA10/data"   // orange pi zero
#define BAIKAL_GPIO_EXIST_CUBE  "/sys/class/gpio_sw/PA19/data"


#ifndef LINUX
static void baikal_detect(void)
{
}
#else

#include <fcntl.h>
#include <termios.h>

static bool detect_one = false;

static void baikal_reset_boards(struct cgpu_info *baikal)
{
    int fd;
    int amount;
    struct baikal_info *info = baikal->device_data;

    if(info->miner_type == BAIKAL_MINER_TYPE_MINI) {
        fd = open(BAIKAL_GPIO_RESET, O_WRONLY);
        if (fd < 0) {
            applog(LOG_INFO, "baikal open : %s fail", BAIKAL_GPIO_RESET);
            return;
        }
    }
    else if(info->miner_type == BAIKAL_MINER_TYPE_CUBE) {
        fd = open(BAIKAL_GPIO_RESET_CUBE, O_WRONLY);
        if (fd < 0) {
            applog(LOG_INFO, "baikal open : %s fail", BAIKAL_GPIO_RESET_CUBE);
            return;
        }
    }
    else {
        return;
    }

    amount = write(fd, "0", 1);
    if (amount < 0) {
        close(fd);
        return;
    }
    cgsleep_ms(10);

    amount = write(fd, "1", 1);
    if (amount < 0) {
        close(fd);
        return;
    }
    close(fd);

    cgsleep_ms(100);    
}

static bool baikal_exist(uint8_t *type)
{
    int fd;
    int amount;
    char value = 0;

    fd = open(BAIKAL_GPIO_EXIST, O_RDONLY);
    if (fd < 0) {
        applog(LOG_INFO, "baikal exist : %s fail", BAIKAL_GPIO_EXIST);
        goto cube;
    }

    amount = read(fd, &value, 1);
    if(amount < 0) {
        close(fd);
        goto cube;
    }
    close(fd);

    if (value != '0') {
        *type = BAIKAL_MINER_TYPE_MINI;
        return true;
    }

cube:
    fd = open(BAIKAL_GPIO_EXIST_CUBE, O_RDONLY);
    if (fd < 0) {
        applog(LOG_INFO, "baikal exist : %s fail", BAIKAL_GPIO_EXIST_CUBE);
        return false;
    }

    amount = read(fd, &value, 1);
    if(amount < 0) {
        close(fd);
        return false;
    }
    close(fd);

    if (value == '0') {
        *type = BAIKAL_MINER_TYPE_CUBE;
        return true;
    }

    return false;
}


static int baikal_init_com(const char *devpath, int baud, uint8_t timeout)
{
    struct termios options;

    int fd = open(devpath, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        return -1;
    }
    
    if (tcgetattr(fd, &options) < 0) {
        return -1;
    }

    cfsetspeed(&options, baud);

    options.c_cflag &= ~(CSIZE | PARENB);
    options.c_cflag |= CS8;
    options.c_cflag |= CREAD;
    options.c_cflag |= CLOCAL;

    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK |
                         ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    options.c_cc[VTIME] = (cc_t)timeout;
    options.c_cc[VMIN]  = 0;

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        return -1;
    }

    return fd;
}


static int baikal_sendmsg(struct cgpu_info *baikal, baikal_msg *msg)
{
    int i, pos = 0;
    int amount;
    uint8_t buf[256] = {0,};

    buf[pos++] = ':';
    buf[pos++] = msg->miner_id;
    buf[pos++] = msg->cmd;
    buf[pos++] = msg->param;
    buf[pos++] = msg->dest;

    for (i = 0; i < msg->len; i++, pos += 2) {
		buf[pos+1] = msg->data[i];
	}

	buf[pos++] = '\r';
	buf[pos++] = '\n';

    amount = write(baikal->fd, buf, pos);    
    if (amount < 0) {
        return amount;
    }

    return amount;
}


static int baikal_readmsg(struct cgpu_info *baikal, baikal_msg *msg, int size)
{
    int amount;
    int len, pos = 1;
    uint8_t buf[128] = {0,};

    amount = read(baikal->fd, buf, size);    
    if (amount < size) {
        return -1;
    }

    if ((buf[0] != ':') || (buf[amount - 2] != '\r') || (buf[amount - 1] != '\n')) {
        return -1;
    }

    msg->miner_id   = buf[pos++];
    msg->cmd        = buf[pos++];
    msg->param      = buf[pos++];
    msg->dest       = buf[pos++];

    for (len = 0; pos < amount - 2; len++, pos += 2) {
		msg->data[len] = buf[pos+1];
	}

    msg->len = len;

    return amount;
}

static void baikal_cleanup(struct cgpu_info *baikal)
{
    int i;
    struct baikal_info *info  = baikal->device_data;
    struct miner_info *miner;
    struct cgpu_info *tmp;
    struct thr_info *thr;
    
    for ( i = 0; i < info->miner_count; i++) {
        miner  = &info->miners[i];
        thr = mining_thr[miner->thr_id];            
        if (thr) {
            tmp = thr->cgpu;
            tmp->shutdown = true;
        }                        
    }

    detect_one = false;
}

static void baikal_clearbuffer(struct cgpu_info *baikal)
{
	int err, retries = 0;
	baikal_msg msg;

	do {        
		err = baikal_readmsg(baikal, &msg, 128);		
		if (err < 0)
			break;
	} while (retries++ < 10);
}


static bool baikal_finalize(struct cgpu_info *baikal)
{    
    close(baikal->fd);

	if(baikal->device_data) {
		free(baikal->device_data);
        baikal->device_data = NULL;
	}

	if(baikal->mutex) {
		free(baikal->mutex);
	}

	if(baikal->name) {
		free(baikal->name);
	}

	free(baikal);    

    return true;
}


static bool baikal_reset(struct cgpu_info *baikal)
{
    int amount;
    struct baikal_info *info    = baikal->device_data;
    baikal_msg msg = {0,};

    msg.miner_id    = 0x0;
    msg.cmd         = BAIKAL_RESET;
    msg.len         = 0;
    msg.dest        = 0;

    mutex_lock(baikal->mutex);

    amount = baikal_sendmsg(baikal, &msg);
    if (amount < 0) {
        mutex_unlock(baikal->mutex);
        return false;
    }

    cgsleep_ms(200);

    amount = baikal_readmsg(baikal, &msg, 7);
    if (amount < 0) {
        mutex_unlock(baikal->mutex);
        return false;
    }

    info->miner_count = msg.param;

    mutex_unlock(baikal->mutex);

    return true;
}


static bool baikal_getinfo(struct cgpu_info *baikal)
{
    int amount;
    baikal_msg msg = {0,};
    struct baikal_info *info    = baikal->device_data;
    struct miner_info *miner  = &info->miners[baikal->miner_id];

    msg.miner_id    = baikal->miner_id;
    msg.cmd         = BAIKAL_GET_INFO;
    msg.dest        = 0;
    msg.len         = 0;

    mutex_lock(baikal->mutex);

    amount = baikal_sendmsg(baikal, &msg);
    if (amount < 0) {
        mutex_unlock(baikal->mutex);
        return false;
    }

    amount = baikal_readmsg(baikal, &msg, 17);
    if (amount < 0) {
        mutex_unlock(baikal->mutex);
        return false;
    }

    mutex_unlock(baikal->mutex);

    miner->fw_ver       = msg.data[0];
    miner->hw_ver       = msg.data[1];
    miner->asic_ver     = msg.data[2];
    miner->clock        = msg.data[3]<<1;
    miner->asic_count   = msg.data[4];
    miner->working_diff = 0.01;
    miner->working      = true;
    miner->overheated   = false;

    return true;
}


static bool baikal_setoption(struct cgpu_info *baikal, uint16_t clk, uint8_t mode, uint8_t temp, uint8_t fanspeed)
{
    int amount;
    baikal_msg msg = {0,};

    msg.miner_id    = baikal->miner_id;
    msg.cmd         = BAIKAL_SET_OPTION;
    msg.data[0]     = (clk == 0 ) ? clk : ((clk/10)%20) + 2;
    msg.data[1]     = mode;
    msg.data[2]     = temp;
    msg.data[3]     = fanspeed;
    msg.dest        = 0;
    msg.len         = 4;

    mutex_lock(baikal->mutex);

    amount = baikal_sendmsg(baikal, &msg);
    if (amount < 0) {
        mutex_unlock(baikal->mutex);
        return false;
    }

    amount = baikal_readmsg(baikal, &msg, 7);
    if (amount < 0) {
        mutex_unlock(baikal->mutex);
        return false;
    }

    mutex_unlock(baikal->mutex);

    return true;
}


static bool baikal_setidle(struct cgpu_info *baikal)
{
    int amount;
    struct baikal_info *info    = baikal->device_data;
    baikal_msg msg = {0,};

    msg.miner_id    = 0x0;
    msg.cmd         = BAIKAL_SET_IDLE;
    msg.len         = 0;
    msg.dest        = 0;

    mutex_lock(baikal->mutex);

    amount = baikal_sendmsg(baikal, &msg);
    mutex_unlock(baikal->mutex);

    return true;
}
static bool baikal_detect_remains(struct cgpu_info *baikal)
{
    int index;
    struct baikal_info *info = baikal->device_data;

    for (index = 1; index < info->miner_count; index++) {
		struct cgpu_info *tmp = calloc(1, sizeof(*tmp));
        tmp->drv            = &baikals_drv;
        tmp->deven          = DEV_ENABLED;
        tmp->threads        = 1;		
        tmp->dev_start_tv.tv_sec = 0;
        tmp->miner_id       = index;

        tmp->device_data    = baikal->device_data;
        tmp->mutex          = baikal->mutex;
        tmp->algorithm      = baikal->algorithm;
        tmp->fd             = baikal->fd;


        if( baikal_getinfo(tmp) == false) {
            free(tmp);
            continue;
        }

        if (baikal_setoption(tmp, info->clock, to_baikal_algorithm(baikal->algorithm.type), info->cutofftemp, info->fanspeed) != true) {
            free(tmp);
            continue;
        }

        if (!add_cgpu(tmp)) {
            free(tmp);
        }        
    }

    return true;
}


static void baikal_detect(void)
{	
    struct cgpu_info *baikal;
    struct baikal_info *info;
    int clock           = BAIKAL_CLK_DEF;
    int cutofftemp      = BAIKAL_CUTOFF_TEMP;
    int fanspeed        = BAIKAL_FANSPEED_DEF;
    int recovertemp     = BAIKAL_RECOVER_TEMP;
    uint8_t miner_type  = BAIKAL_MINER_TYPE_NONE;
    
    if (detect_one == true) {
        return;
    }    

    if(baikal_exist(&miner_type) != true) {
        return; 
    }

    if(opt_baikal_options != NULL) {
		sscanf(opt_baikal_options, "%d:%d:%d", &clock, &recovertemp, &cutofftemp);
        if (clock < BAIKAL_CLK_MIN) {
            clock = BAIKAL_CLK_MIN;
        }
        if (clock > BAIKAL_CLK_MAX) {
            clock = BAIKAL_CLK_MAX;
        }
	}

    if(opt_baikal_fan != NULL) {
        sscanf(opt_baikal_fan, "%d", &fanspeed);
        if (fanspeed > BAIKAL_FANSPEED_MAX) {
            fanspeed = BAIKAL_FANSPEED_DEF;
        }
    }

   	baikal = calloc(1, sizeof(*baikal));
    baikal->drv     = &baikals_drv;
	baikal->deven   = DEV_ENABLED;
	baikal->threads = 1;
    baikal->dev_start_tv.tv_sec = 0;

    baikal->mutex   = calloc(1, sizeof(*(baikal->mutex)));
    mutex_init(baikal->mutex);

    info = (struct baikal_info*)calloc(1, sizeof(struct baikal_info));    
    info->clock         = clock;
    info->cutofftemp    = (uint8_t)cutofftemp;
    info->fanspeed      = (uint8_t)fanspeed;
	info->recovertemp   = (uint8_t)recovertemp;
    info->miner_type    = miner_type;

    baikal->device_data = info;
    baikal->name        = strdup("BKLS");
    baikal->miner_id    = 0;
    baikal->algorithm   = default_profile.algorithm; 

    baikal->fd = baikal_init_com(BAIKAL_IO_PORT, BAIKAL_IO_SPEED, 30);
    if (baikal->fd < 0) {
        goto out;
    }    

    baikal_reset_boards(baikal);

    if (baikal_reset(baikal) != true) {
        goto out;
    }

    if (baikal_getinfo(baikal) != true) {
        applog(LOG_ERR, "baikal_getinfo error");
        goto out;
    }

    if (baikal_setoption(baikal, clock, to_baikal_algorithm(default_profile.algorithm.type), cutofftemp, fanspeed) != true) {
        applog(LOG_ERR, "baikal_getinfo error");
        goto out;
    }

    if (!add_cgpu(baikal)) {
        goto out;
    }
    
    baikal_detect_remains(baikal);   

    detect_one = true;
    return;

out:
    baikal_finalize(baikal);
	return;
}


static void baikal_get_statline_before(char *buf, size_t bufsiz, struct cgpu_info *baikal)
{
    struct baikal_info *info    = baikal->device_data;
    struct miner_info *miner    = &info->miners[baikal->miner_id];

    tailsprintf(buf, bufsiz, "%s%dC %3uMHz [ASICS #%d] | ",(baikal->temp < 10) ? " " : "", (int)miner->temp, miner->clock, miner->asic_count);
}


static struct api_data* baikal_api_stats(struct cgpu_info *cgpu)
{
	struct baikal_info *info    = cgpu->device_data;
    struct miner_info *miner    = &info->miners[cgpu->miner_id];
    struct thr_info *thr =       mining_thr[miner->thr_id];
	struct api_data *root       = NULL;

    root = api_add_int(root, "Chip Count", (int*)&miner->asic_count, false);
    root = api_add_int(root, "Clock", (int*)&miner->clock, false);
    root = api_add_int(root, "HWV", (int*)&miner->hw_ver, false); 
    root = api_add_int(root, "FWV", (int*)&miner->fw_ver, false); 
    root = api_add_string(root, "Algo", (char *)algorithm_type_str[thr->cgpu->algorithm.type], false);

	return (root);
}


static void baikal_identify(struct cgpu_info *baikal)
{
    int amount;
    baikal_msg msg = {0,};

    msg.miner_id    = baikal->miner_id;
    msg.cmd         = BAIKAL_SET_ID;
    msg.dest        = 0;
    msg.len         = 0;

    mutex_lock(baikal->mutex);

    amount = baikal_sendmsg(baikal, &msg);
    if (amount < 0) {
        mutex_unlock(baikal->mutex);
        return;
    }

    amount = baikal_readmsg(baikal, &msg, 7);
    if (amount < 0) {
        mutex_unlock(baikal->mutex);
        return;
    }

    mutex_unlock(baikal->mutex);
}


static bool baikal_prepare(struct thr_info *thr)
{
    struct cgpu_info *baikal    = thr->cgpu;
    struct baikal_info *info    = baikal->device_data;

    cglock_init(&(info->pool.data_lock));

    return (true);
}


static void baikal_checknonce(struct cgpu_info *baikal, baikal_msg *msg)
{
    struct baikal_info *info = baikal->device_data;
    struct miner_info *miner = &info->miners[msg->miner_id];
    uint8_t work_idx, chip_id;
    uint32_t nonce;    
    
    chip_id     = msg->data[4];
    work_idx    = msg->data[5];    
    nonce       = *((uint32_t *)msg->data);

    /* check algorithm */                                                     
    if (info->works[work_idx].pool->algorithm.type != baikal->algorithm.type){
        return;
    }
                                                                                        
    if (submit_nonce(mining_thr[miner->thr_id], &info->works[work_idx], nonce) == true) {
        miner->nonce++;                                                                  
    } else {                                                                             
        miner->error++;                                                                  
    }                                                                                    
}


static bool baikal_send_work(struct cgpu_info *baikal, int miner_id)
{    
    struct baikal_info *info = baikal->device_data;
    struct miner_info *miner = &info->miners[miner_id];
    struct thr_info *thr = mining_thr[miner->thr_id];
    struct work *work;
    uint32_t target;
    baikal_msg msg;

    /* Do not send */
    if (miner->overheated == true) {
        return true;
    }

    work = get_work(thr, miner->thr_id);
    memcpy(&info->works[info->work_idx],work,sizeof(struct work));

    work->device_diff = MAX(miner->working_diff, work->work_difficulty);
	set_target(work->device_target, work->device_diff, work->pool->algorithm.diff_multiplier2, work->thr_id);

    memcpy(&msg.data[0], work->data, 80);
    target = *((uint32_t *)(&work->device_target[28]));
    memcpy(&msg.data[80], &target, 4);
    
    if (work->pool->algorithm.type != thr->cgpu->algorithm.type) {
        thr->cgpu->algorithm.type = work->pool->algorithm.type;
    }

    msg.data[84]    = to_baikal_algorithm(thr->cgpu->algorithm.type); 
    msg.data[85]    = 0xFF;
    
    msg.miner_id    = miner_id;
    msg.cmd         = BAIKAL_SEND_WORK;
    msg.param       = info->work_idx;
    msg.dest        = 0;
    msg.len         = 86;

    mutex_lock(baikal->mutex);

    if (baikal_sendmsg(baikal, &msg) < 0) {
        applog(LOG_ERR, "baikal_send_work : sendmsg error[%d]", miner_id);
        mutex_unlock(baikal->mutex);
        return false;
    }    

    if (baikal_readmsg(baikal, &msg, 7) < 0) {
        applog(LOG_ERR, "baikal_send_work : readmsg error[%d]", miner_id);
        mutex_unlock(baikal->mutex);
        return false;
    }

    /* update clock */
    miner->clock = msg.param<<1;

    info->work_idx++;
    if (info->work_idx >= BAIKAL_WORK_FIFO) {
        info->work_idx = 0;
    }
    cgtimer_time(&miner->start_time);
    mutex_unlock(baikal->mutex);

    return true;
}


static bool baikal_process_result(struct cgpu_info* baikal)
{
    struct baikal_info *info = baikal->device_data;    
    struct miner_info *miner;
    baikal_msg msg = {0,};
    int i;

    for (i = 0; i < info->miner_count; i++) {
        msg.miner_id    = i; 
        msg.cmd         = BAIKAL_GET_RESULT;
        msg.dest        = 0;
        msg.len         = 0;
        
        mutex_lock(baikal->mutex);
        if( baikal_sendmsg(baikal, &msg) < 0 ) {
            applog(LOG_ERR, "baikal_process_result : sendmsg error");
            mutex_unlock(baikal->mutex);
            return false;
        }

        if( baikal_readmsg(baikal, &msg, 23) < 0 ) {
            applog(LOG_ERR, "baikal_process_result : readmsg error");
            mutex_unlock(baikal->mutex);
            return false;
        }
        mutex_unlock(baikal->mutex);

        miner = &info->miners[i];
        miner->temp = msg.data[6];
        switch (msg.param) {
        case 1:     /* nonce detect */
            baikal_checknonce(baikal, &msg);
            break;
        case 2:     /* job empty */
            baikal_send_work(baikal, i);
            break;
        case 3:     /* new miner detect*/
            baikal_cleanup(baikal);
            break;
        case 0:
        default:
            break;
        }

        if (miner->temp > info->cutofftemp) {
            miner->overheated = true;
        }
        else if (miner->temp < info->recovertemp) {
            miner->overheated = false;
        }

        cgsleep_ms(10);
    }
    
    return true;
}


static int64_t baikal_scanhash(struct thr_info *thr)
{
    struct cgpu_info *baikal = thr->cgpu;
    struct baikal_info *info = baikal->device_data;        
    struct miner_info *miner = &info->miners[baikal->miner_id];
    cgtimer_t now;
    int elapsed, i;

    if (baikal->usbinfo.nodev) {
        return -1;
    }    

    if (baikal->miner_id == 0) {
        if(baikal_process_result(baikal) != true) {
            baikal_cleanup(baikal);
            return -1;
        }
    }
    else {
        cgsleep_ms(100);
    }    

    baikal->temp = miner->temp;

    cgtimer_time(&now);
    elapsed = cgtimer_to_ms(&now) - cgtimer_to_ms(&miner->start_time);
    miner->start_time = now;

    return (miner->clock * miner->asic_count * elapsed * 128);
}


static void baikal_update_work(struct cgpu_info *baikal)
{
    int i;
    struct timeval now;    
    struct baikal_info *info = baikal->device_data;
    struct miner_info *miner = &info->miners[baikal->miner_id];
    struct thr_info *thr = mining_thr[miner->thr_id];

    thr->work_update    = false;
    //thr->work_restart   = false;

    if (baikal->miner_id == 0) {
        for (i = 0; i < info->miner_count; i++) {
            if (baikal_send_work(baikal, i) != true) {
                baikal_cleanup(baikal);
            }
        }     
    }
}


static bool baikal_init(struct thr_info *thr)
{
    struct cgpu_info *baikal    = thr->cgpu;
    struct baikal_info *info    = baikal->device_data;
    struct miner_info *miner    = &info->miners[baikal->miner_id];

    miner->thr_id               = thr->id;
    cgtimer_time(&miner->start_time);
	return (true);
}


static void baikal_shutdown(struct thr_info *thr)
{
    struct cgpu_info *baikal = thr->cgpu;
	struct baikal_info *info = baikal->device_data;

    baikal->shutdown = true;
    if (baikal->miner_id == 0) {
        close(baikal->fd);
    }
}
#endif 


struct device_drv baikals_drv = {
    .drv_id				    = DRIVER_baikals,
    .dname					= "Baikal",
    .name					= "BKLS",
    .drv_detect				= baikal_detect,
#ifdef LINUX
    .get_statline_before	= baikal_get_statline_before,
    .get_api_stats			= baikal_api_stats,
    .identify_device		= baikal_identify,    
    .thread_prepare			= baikal_prepare,
	.thread_init			= baikal_init,
    .hash_work              = hash_driver_work,
    .update_work            = baikal_update_work,
    //.flush_work             = baikal_update_work,
    .scanwork				= baikal_scanhash,
    .thread_shutdown        = baikal_shutdown,
#endif 
};


