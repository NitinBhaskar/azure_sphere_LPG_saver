#include <stdint.h>
#include "grid_eye.h"

int32_t grid_eye_reset(ge_ctx_t* ctx)
{
	int32_t ret;
	uint8_t reg = GE_RST, data = 0x3F;
	ret = ctx->write_reg(ctx->handle, reg, &data, 1);
	if (ret < 0) {
		return ret;
	}
	reg = GE_PCTL; data = 0x00;
	ret = ctx->write_reg(ctx->handle, reg, &data, 1);
	if (ret < 0) {
		return ret;
	}
	reg = GE_FPSC; data = 0x01;
	ret = ctx->write_reg(ctx->handle, reg, &data, 1);
	return ret;
}

int32_t grid_eye_read_thermistor(ge_ctx_t* ctx, uint16_t* val)
{
	int32_t ret;
	uint8_t reg = GE_TTHL;
	ret = ctx->read_reg(ctx->handle, reg, (uint8_t*)val, 2);
	return ret;
}

int32_t grid_eye_read_heatmap(ge_ctx_t* ctx, uint16_t* val)
{
	int32_t ret;
	uint8_t reg = GE_DATA01L;
	uint8_t index;
	uint8_t* data = (uint8_t *)val;
	for (index = 0; index < (GE_TOTAL_PIXELS * 2); index++)
	{
		ret = ctx->read_reg(ctx->handle, (reg + index), (data + index), 1);
		if (ret < 0)
		{
			return ret;
		}
	}
	return ret;
}