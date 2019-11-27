#pragma once
#ifndef __GRID_EYE_H_
#define __GRID_EYE_H_

#ifdef __cplusplus
extern "C" {
#endif

#define GE_ADDRESS	0x68 

#define GE_PCTL     0x00
#define GE_RST      0x01
#define GE_FPSC     0x02
#define GE_TTHL     0x0E
#define GE_DATA01L  0x80 /* Pixel0 low data */

#define GE_TOTAL_PIXELS (8*8)

typedef int32_t(*ge_write_ptr)(void*, uint8_t, uint8_t*, uint16_t);
typedef int32_t(*ge_read_ptr) (void*, uint8_t, uint8_t*, uint16_t);

typedef struct {
	/** Component mandatory fields **/
	ge_write_ptr  write_reg;
	ge_read_ptr   read_reg;
	/** Customizable optional pointer **/
	void* handle;
} ge_ctx_t;

int32_t grid_eye_reset(ge_ctx_t* ctx);
int32_t grid_eye_read_thermistor(ge_ctx_t* ctx, uint16_t* val);
int32_t grid_eye_read_heatmap(ge_ctx_t* ctx, uint16_t* val);

#ifdef __cplusplus
}
#endif 
#endif /* __GRID_EYE_H_ */