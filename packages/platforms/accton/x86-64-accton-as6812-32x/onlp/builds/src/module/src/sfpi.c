/************************************************************
 * <bsn.cl fy=2014 v=onl>
 *
 *           Copyright 2014 Big Switch Networks, Inc.
 *           Copyright 2015 Accton Technology Corporation.
 *
 * Licensed under the Eclipse Public License, Version 1.0 (the
 * "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at
 *
 *        http://www.eclipse.org/legal/epl-v10.html
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the
 * License.
 *
 * </bsn.cl>
 ************************************************************
 *
 *
 *
 ***********************************************************/
#include <onlp/platformi/sfpi.h>

#include <fcntl.h> /* For O_RDWR && open */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "platform_lib.h"

#define MAX_SFP_PATH 64
static char sfp_node_path[MAX_SFP_PATH] = {0};
#define FRONT_PORT_TO_CPLD_MUX_INDEX(port) (port+2)

static int 
as6812_32x_sfp_node_read_int(char *node_path, int *value, int data_len)
{
    int ret = 0;
    char buf[8];    
    *value = 0;

    ret = deviceNodeReadString(node_path, buf, sizeof(buf), data_len);

    if (ret == 0) {
        *value = atoi(buf);
    }

    return ret;
}

static char* 
as6812_32x_sfp_get_port_path(int port, char *node_name)
{
    sprintf(sfp_node_path, "/sys/bus/i2c/devices/%d-0050/%s", 
                           FRONT_PORT_TO_CPLD_MUX_INDEX(port),
                           node_name);

    return sfp_node_path;
}

/************************************************************
 *
 * SFPI Entry Points
 *
 ***********************************************************/
int
onlp_sfpi_init(void)
{
    /* Called at initialization time */    
    return ONLP_STATUS_OK;
}

int
onlp_sfpi_bitmap_get(onlp_sfp_bitmap_t* bmap)
{
    /*
     * Ports {0, 32}
     */
    int p;
    AIM_BITMAP_INIT(bmap, 64);
    AIM_BITMAP_CLR_ALL(bmap);
    
    for(p = 0; p < 32; p++) {
        AIM_BITMAP_SET(bmap, p);
    }

    return ONLP_STATUS_OK;
}

int
onlp_sfpi_is_present(int port)
{
    /*
     * Return 1 if present.
     * Return 0 if not present.
     * Return < 0 if error.
     */
    int present;
    char* path = as6812_32x_sfp_get_port_path(port, "sfp_is_present");

    if (as6812_32x_sfp_node_read_int(path, &present, 1) != 0) {
        AIM_LOG_ERROR("Unable to read present status from port(%d)\r\n", port);
        return ONLP_STATUS_E_INTERNAL;
    }
    
    return present;
}

int
onlp_sfpi_presence_bitmap_get(onlp_sfp_bitmap_t* dst)
{
    uint32_t bytes[4];
    char* path;
    FILE* fp;

    path = as6812_32x_sfp_get_port_path(0, "sfp_is_present_all");
    fp = fopen(path, "r");
    
    if(fp == NULL) {
        AIM_LOG_ERROR("Unable to open the sfp_is_present_all device file.");
        return ONLP_STATUS_E_INTERNAL;
    }
    int count = fscanf(fp, "%x %x %x %x",
                       bytes+0,
                       bytes+1,
                       bytes+2,
                       bytes+3
                       );
    fclose(fp);
    if(count != AIM_ARRAYSIZE(bytes)) {
        /* Likely a CPLD read timeout. */
        AIM_LOG_ERROR("Unable to read all fields from the sfp_is_present_all device file.");
        return ONLP_STATUS_E_INTERNAL;
    }

    /* Convert to 64 bit integer in port order */
    int i = 0;
    uint32_t presence_all = 0 ;
    for(i = AIM_ARRAYSIZE(bytes)-1; i >= 0; i--) {
        presence_all <<= 8;
        presence_all |= bytes[i];
    }

    /* Populate bitmap */
    for(i = 0; presence_all; i++) {
        AIM_BITMAP_MOD(dst, i, (presence_all & 1));
        presence_all >>= 1;
    }

    return ONLP_STATUS_OK;
}

int
onlp_sfpi_rx_los_bitmap_get(onlp_sfp_bitmap_t* dst)
{
    return ONLP_STATUS_OK;
}

int
onlp_sfpi_eeprom_read(int port, uint8_t data[256])
{
    char* path = as6812_32x_sfp_get_port_path(port, "sfp_eeprom");

    /*
     * Read the SFP eeprom into data[]
     *
     * Return MISSING if SFP is missing.
     * Return OK if eeprom is read
     */
    memset(data, 0, 256);
    
    if (deviceNodeReadBinary(path, (char*)data, 256, 256) != 0) {
        AIM_LOG_ERROR("Unable to read eeprom from port(%d)\r\n", port);
        return ONLP_STATUS_E_INTERNAL;
    }

    return ONLP_STATUS_OK;
}

int
onlp_sfpi_denit(void)
{
    return ONLP_STATUS_OK;
}
