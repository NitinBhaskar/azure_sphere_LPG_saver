/*
 *  Some of the code in this file was copied from ST Micro.  Below is their required information.
 *
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"

#include <applibs/log.h>
#include <applibs/i2c.h>

#include "mt3620_avnet_dev.h"
#include "deviceTwin.h"
#include "azure_iot_utilities.h"
#include "build_options.h"
#include "i2c.h"
#include "grid_eye.h"

#define HEAT_THRESHOLD 300

/* Private variables ---------------------------------------------------------*/


static int sensorTimerFd = -1;
ge_ctx_t dev_ctx;

//Extern variables
int i2cFd = -1;
extern int epollFd;
extern volatile sig_atomic_t terminationRequired;

static uint16_t heat_map1[GE_TOTAL_PIXELS];
static uint16_t heat_map2[GE_TOTAL_PIXELS];

static uint16_t * curr_heat_map = heat_map1;
static uint16_t * prev_heat_map = NULL;

//Private functions

// Routines to read/write to the LSM6DSO device
static int32_t platform_write(int *fD, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_read(int *fD, uint8_t reg, uint8_t *bufp, uint16_t len);

/// <summary>
///     Sleep for delayTime ms
/// </summary>
void HAL_Delay(int delayTime) {
	struct timespec ts;
	ts.tv_sec = 0;
	ts.tv_nsec = delayTime * 10000;
	nanosleep(&ts, NULL);
}

/// <summary>
///     Print latest data from on-board sensors.
/// </summary>
void SensorTimerEventHandler(EventData *eventData)
{
	int32_t ret;
	uint16_t index;
	uint16_t match_val = 0;
	uint16_t* ptr;

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
	static bool firstPass = true;
#endif
	// Consume the event.  If we don't do this we'll come right back 
	// to process the same event again
	if (ConsumeTimerFdEvent(sensorTimerFd) != 0) {
		terminationRequired = true;
		return;
	}

	ret = grid_eye_read_heatmap(&dev_ctx, curr_heat_map);
	if (ret < 0)
	{
		Log_Debug("ERROR: grid_eye_read_heatmap: errno=%d (%s)\n", errno, strerror(errno));
		return;
	}
	/* Don't do anything for 1st read */
	if (prev_heat_map == NULL)
	{
		prev_heat_map = curr_heat_map;
		return;
	}

	for (index = 1; index < GE_TOTAL_PIXELS; index++)
	{
		if (prev_heat_map[index] == curr_heat_map[index])
		{
			if (prev_heat_map[index] > HEAT_THRESHOLD)
			{
				if (match_val < prev_heat_map[index])
				{
					match_val = prev_heat_map[index];
				}
			}
		}
	}

	ptr = prev_heat_map;
	prev_heat_map = curr_heat_map;
	curr_heat_map = ptr;

	if (match_val == 0)
	{
		return;
	}

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))

		// We've seen that the first read of the Accelerometer data is garbage.  If this is the first pass
		// reading data, don't report it to Azure.  Since we're graphing data in Azure, this data point
		// will skew the data.
		if (!firstPass) {

			// Allocate memory for a telemetry message to Azure
			char *pjsonBuffer = (char *)malloc(JSON_BUFFER_SIZE);
			if (pjsonBuffer == NULL) {
				Log_Debug("ERROR: not enough memory to send telemetry");
			}

			// construct the telemetry message
			snprintf(pjsonBuffer, JSON_BUFFER_SIZE, "{\"MAX_TEMP\":\"%d\"}", match_val);
			Log_Debug("%s\n", pjsonBuffer);

			Log_Debug("\n[Info] Sending telemetry: %s\n", pjsonBuffer);
			AzureIoT_SendMessage(pjsonBuffer);
			free(pjsonBuffer);

		}

		firstPass = false;

#endif 

}

/// <summary>
///     Initializes the I2C interface.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
int initI2c(void) {

	// Begin MT3620 I2C init 
	i2cFd = I2CMaster_Open(MT3620_RDB_HEADER4_ISU2_I2C);
	if (i2cFd < 0) {
		Log_Debug("ERROR: I2CMaster_Open: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	int result = I2CMaster_SetBusSpeed(i2cFd, I2C_BUS_SPEED_STANDARD);
	if (result != 0) {
		Log_Debug("ERROR: I2CMaster_SetBusSpeed: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	result = I2CMaster_SetTimeout(i2cFd, 100);
	if (result != 0) {
		Log_Debug("ERROR: I2CMaster_SetTimeout: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}
	// Initialize lsm6dso mems driver interface
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &i2cFd;

	result = grid_eye_reset(&dev_ctx);
	if (result != 0) {
		Log_Debug("ERROR: Grideye reset error: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	// Define the period in the build_options.h file
	struct timespec accelReadPeriod = { .tv_sec = SENSOR_READ_PERIOD_SECONDS,.tv_nsec = SENSOR_READ_PERIOD_NANO_SECONDS };
	// event handler data structures. Only the event handler field needs to be populated.
	static EventData accelEventData = { .eventHandler = &SensorTimerEventHandler };
	sensorTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &accelReadPeriod, &accelEventData, EPOLLIN);
	if (sensorTimerFd < 0) {
		return -1;
	}

	return 0;
}

/// <summary>
///     Closes the I2C interface File Descriptors.
/// </summary>
void closeI2c(void) {

	CloseFdAndPrintError(i2cFd, "i2c");
}

/// <summary>
///     Writes data to the lsm6dso i2c device
/// </summary>
/// <returns>0</returns>

static int32_t platform_write(int *fD, uint8_t reg, uint8_t *bufp,
	uint16_t len)
{

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("platform_write()\n");
	Log_Debug("reg: %0x\n", reg);
	Log_Debug("len: %0x\n", len);
	Log_Debug("bufp contents: ");
	for (int i = 0; i < len; i++) {

		Log_Debug("%0x: ", bufp[i]);
	}
	Log_Debug("\n");
#endif 

	// Construct a new command buffer that contains the register to write to, then the data to write
	uint8_t cmdBuffer[len + 1];
	cmdBuffer[0] = reg;
	for (int i = 0; i < len; i++) {
		cmdBuffer[i + 1] = bufp[i];
	}

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("cmdBuffer contents: ");
	for (int i = 0; i < len + 1; i++) {

		Log_Debug("%0x: ", cmdBuffer[i]);
	}
	Log_Debug("\n");
#endif

	// Write the data to the device
	int32_t retVal = I2CMaster_Write(*fD, GE_ADDRESS, cmdBuffer, (size_t)len + 1);
	if (retVal < 0) {
		Log_Debug("ERROR: platform_write: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}
#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("Wrote %d bytes to device.\n\n", retVal);
#endif
	return 0;
}

/// <summary>
///     Reads generic device register from the i2c interface
/// </summary>
/// <returns>0</returns>

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(int *fD, uint8_t reg, uint8_t *bufp,
	uint16_t len)
{

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("platform_read()\n");
	Log_Debug("reg: %0x\n", reg);
	Log_Debug("len: %d\n", len);
;
#endif

	// Set the register address to read
	int32_t retVal = I2CMaster_Write(i2cFd, GE_ADDRESS, &reg, 1);
	if (retVal < 0) {
		Log_Debug("ERROR: platform_read(write step): errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	// Read the data into the provided buffer
	retVal = I2CMaster_Read(i2cFd, GE_ADDRESS, bufp, len);
	if (retVal < 0) {
		Log_Debug("ERROR: platform_read(read step): errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("Read returned: ");
	for (int i = 0; i < len; i++) {
		Log_Debug("%0x: ", bufp[i]);
	}
	Log_Debug("\n\n");
#endif 	   

	return 0;
}

