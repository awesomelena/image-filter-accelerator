#include "xparameters.h"
#include <xil_io.h>
#include <xil_printf.h>
#include <xstatus.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "xaxidma.h"
#include "xinterrupt_wrap.h"
#include <xil_cache.h>
#include "xil_util.h"
#include "xtmrctr.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

typedef struct ImageShape {
    u16 Width;
    u16 Height; 
    u16 NumOfPlanes;  
} ImageShape;

typedef struct FilterParams {
    ImageShape Img;
    u16 Radius;
    u16 CoeffScale;
    u16 Ctrl;
    s16 Coeffs[81]; 
} FilterParams;

// functions
static int DmaConfigure(XAxiDma_Config* AxiDmaConfigPtr, XAxiDma* AxiDmaPtr);
static int DmaStartTransfers(XAxiDma* AxiDmaPtr, u8* TxBuffer, u32 TxSize, u8* RxBuffer, u32 RxSize);
static int DmaWaitTransfers(volatile u32* TxFlag, volatile u32* RxFlag, u32 Timeout);
static int AccConfigure(UINTPTR BaseAddress, FilterParams Params);

static void FilterImageSW(u8* DataBuffer, u16* ResultBuffer, FilterParams Params);
static int  FilterImageHW(u8* DataBuffer, u16* ResultBuffer, FilterParams Params);
static int  CheckData(u16* ResultBuffer, u16* ReferentBuffer, FilterParams Params);

static void TxIntrHandler(void *Callback);
static void RxIntrHandler(void *Callback);

// filters
static void CalculateScaleAndQuantize(float* float_coeffs, int n_coeffs, FilterParams* Params);
static void GenerateBoxFilter(FilterParams* Params);
static void GenerateGaussianFilter(FilterParams* Params, float sigma);
static void GenerateLoGFilter(FilterParams* Params, float sigma);
static void GenerateSharpenFilter(FilterParams* Params, float sigma, int use_gaussian, float k);
static void GenerateSobelFilter(FilterParams* Params, int axis);
static void GeneratePrewittFilter(FilterParams* Params, int axis);


#define DMA_TRANSFER_TIMEOUT 1000000 

#define REG_CTRL_ADDR        0x00
#define REG_RADIUS_ADDR      0x04
#define REG_IMG_W_ADDR       0x08
#define REG_IMG_H_ADDR       0x0C
#define REG_COEFF_SCALE_ADDR 0x10
#define REG_COEFF_W0_ADDR    0x14   

static XAxiDma AxiDma;
static XTmrCtr TimerInst;
volatile u32 TxDone;
volatile u32 RxDone;


int main(void)
{
    int Status;
    u8  *DataBuffer = NULL;
    u16 *ReferentBuffer = NULL;
    u16 *ResultBuffer = NULL;

    FilterParams Params;
    int option;
    int running = 1;

    xil_printf("\r\n--- Entering Hardware Accelerator System --- \r\n");

    Status = XTmrCtr_Initialize(&TimerInst, XPAR_AXI_TIMER_0_BASEADDR);
    if (Status != XST_SUCCESS) {
        xil_printf("ERROR: Timer initialization failed\r\n");
        return XST_FAILURE;
    }

    while (running) {
        xil_printf("  Choose filter type:\r\n");
        xil_printf("  1 - Box Filter\r\n");
        xil_printf("  2 - Gaussian Filter\r\n");
        xil_printf("  3 - LoG (Laplacian of Gaussian)\r\n");
        xil_printf("  4 - Sharpening (Box base)\r\n");
        xil_printf("  5 - Sharpening (Gaussian base)\r\n");
        xil_printf("  6 - Sobel\r\n");
        xil_printf("  7 - Prewitt\r\n");
        xil_printf("  8 - Bypass\r\n");
        xil_printf("  0 - Exit\r\n");
        xil_printf("Choice: ");
        scanf("%d", &option);
        xil_printf("%d\r\n", option);

        if (option == 0) {
            running = 0;
            continue;
        }

        if (option < 1 || option > 9) {
            xil_printf("Unknown option.\r\n");
            continue;
        }

        int radius = 1;
        xil_printf("Enter filter radius (0-4): ");
        scanf("%d", &radius);
        xil_printf("%d\r\n", radius);
        Params.Radius = (u16)radius;

        float sigma = 1.0f;
        float k_factor = 1.0f;
        int axis = 0;

        // gathering extra params based on filter selection
        if (option == 2 || option == 3 || option == 5) {
            xil_printf("Enter Sigma value (e.g., 1.0): ");
            scanf("%f", &sigma);
            xil_printf("%d.%02d\r\n", (int)sigma, (int)(sigma*100)%100);
        }
        if (option == 4 || option == 5) {
            xil_printf("Enter K sharpening factor (e.g., 1.0): ");
            scanf("%f", &k_factor);
            xil_printf("%d.%02d\r\n", (int)k_factor, (int)(k_factor*100)%100);
        }
        if (option == 6 || option == 7) {
            xil_printf("Enter Axis (0 = x, 1 = y): ");
            scanf("%d", &axis);
            xil_printf("%d\r\n", axis);
        }

		// mode logic
        int q97_mode = 0;
        xil_printf("Output Mode (0 = 8-bit UInt, 1 = 16-bit Q9.7): ");
        scanf("%d", &q97_mode);
        xil_printf("%d\r\n", q97_mode);

        Params.Ctrl = 0x0000;
        Params.Ctrl |= (q97_mode & 0x01);              // bit 0: Mode
        Params.Ctrl |= (option == 9 ? 0x02 : 0x00);    // bit 1: Bypass

        // generate coefficients
        for(int i=0; i<81; i++) Params.Coeffs[i] = 0;
        Params.CoeffScale = 4096; // default to 1.0

        if (option != 8) {
            switch(option) {
                case 1: GenerateBoxFilter(&Params); break;
                case 2: GenerateGaussianFilter(&Params, sigma); break;
                case 3: GenerateLoGFilter(&Params, sigma); break;
                case 4: GenerateSharpenFilter(&Params, 0.0f, 0, k_factor); break;
                case 5: GenerateSharpenFilter(&Params, sigma, 1, k_factor); break;
                case 6: GenerateSobelFilter(&Params, axis); break;
                case 7: GeneratePrewittFilter(&Params, axis); break;
            }
        }

        // input dimensions
        int w, h, planes;
        xil_printf("Enter Image Width: ");
        scanf("%d", &w); xil_printf("%d\r\n", w);
        
        xil_printf("Enter Image Height: ");
        scanf("%d", &h); xil_printf("%d\r\n", h);
        
        xil_printf("Enter Number of Planes (1=Grayscale, 3=RGB): ");
        scanf("%d", &planes); xil_printf("%d\r\n", planes);

        Params.Img.Width = (u16)w;
        Params.Img.Height = (u16)h;
        Params.Img.NumOfPlanes = (u16)planes;

        // determine output size
        u32 OutWidth  = Params.Img.Width - 2 * Params.Radius;
        u32 OutHeight = Params.Img.Height - 2 * Params.Radius;
        
        u32 ImgSizeBytes = Params.Img.Width * Params.Img.Height * Params.Img.NumOfPlanes * sizeof(u8);
        u32 OutSizeHWords = OutWidth * OutHeight * Params.Img.NumOfPlanes * sizeof(u16);

        xil_printf("\r\n>> Processing Setup: In(%dx%d), Out(%dx%d), Planes=%d <<\r\n", 
            Params.Img.Width, Params.Img.Height, OutWidth, OutHeight, Params.Img.NumOfPlanes);

        xil_printf(">> Automatically calculated optimal CoeffScale: %d (Q4.12 format) <<\r\n", Params.CoeffScale);

        // memory allocation
        DataBuffer = (u8*)malloc(ImgSizeBytes);
        ResultBuffer = (u16*)malloc(OutSizeHWords);
        ReferentBuffer = (u16*)malloc(OutSizeHWords);

        if (!DataBuffer || !ResultBuffer || !ReferentBuffer) {
            xil_printf("ERROR: Memory allocation failed!\r\n");
            if(DataBuffer) free(DataBuffer);
            if(ResultBuffer) free(ResultBuffer);
            if(ReferentBuffer) free(ReferentBuffer);
            continue;
        }

        xil_printf("\r\nData Buffers Allocated:\r\n");
        xil_printf("DataBuffer     Addr: 0x%08X  (Size: %d bytes)\r\n", DataBuffer, ImgSizeBytes);
        xil_printf("ResultBuffer   Addr: 0x%08X  (Size: %d bytes)\r\n", ResultBuffer, OutSizeHWords);
		xil_printf("ReferentBuffer Addr: 0x%08X  (Size: %d bytes)\r\n", ReferentBuffer, OutSizeHWords);

        // loading data via debug console
        xil_printf("\r\n Action Required! \r\n");
        xil_printf("Load input image to RAM using the following XDB command:\r\n");
        xil_printf("mwr -size b -bin -file \"your_path/image.bin\" 0x%08X %d\r\n", DataBuffer, ImgSizeBytes);
        
        xil_printf("\r\nOnce loaded, type '1' and press Enter to continue: ");
        int ready;
        scanf("%d", &ready);

        // hardware configuration
        Status = AccConfigure(XPAR_ACC_IMAGE_FILTER_0_BASEADDR, Params);
        if (Status != XST_SUCCESS) {
            xil_printf("ERROR: Accelerator configuration failed\r\n");
            goto cleanup;
        }

        Xil_DCacheInvalidateRange((UINTPTR)DataBuffer, ImgSizeBytes);

        // process sw
        xil_printf("\r\nRunning Software processing (Generating referent)...\r\n");

        XTmrCtr_Reset(&TimerInst, 0);
        XTmrCtr_Start(&TimerInst, 0);

        FilterImageSW(DataBuffer, ReferentBuffer, Params);

        XTmrCtr_Stop(&TimerInst, 0);
        u32 sw_ticks = XTmrCtr_GetValue(&TimerInst, 0);

        double sw_time_us = (double)sw_ticks / ((double)XPAR_AXI_TIMER_0_CLOCK_FREQUENCY / 1000000.0);

        // process hw
        xil_printf("Running Hardware processing...\r\n");

        XTmrCtr_Reset(&TimerInst, 0);
        XTmrCtr_Start(&TimerInst, 0);

        Status = FilterImageHW(DataBuffer, ResultBuffer, Params);

        XTmrCtr_Stop(&TimerInst, 0);
        u32 hw_ticks = XTmrCtr_GetValue(&TimerInst, 0);

        if (Status != XST_SUCCESS) {
            xil_printf("ERROR: Hardware processing failed\r\n");
            goto cleanup;
        }

        double hw_time_us = (double)hw_ticks / ((double)XPAR_AXI_TIMER_0_CLOCK_FREQUENCY / 1000000.0);

        // verifying output
        Status = CheckData(ResultBuffer, ReferentBuffer, Params);
        if (Status == XST_SUCCESS) {
            xil_printf("SUCCESS: Hardware and Software results match perfectly!\r\n");
        }

        // double speedup = sw_time_us / hw_time_us;
        xil_printf("\r\nTIMING RESULTS:\r\n");
        xil_printf("Software Execution Time: %d us\r\n", (int)sw_time_us);
        xil_printf("Hardware Execution Time: %d us\r\n", (int)hw_time_us);

        // output instruction
        xil_printf("\r\n Action Required! \r\n");
        xil_printf("Save the resulting image from RAM using the following XDB command:\r\n");
        xil_printf("mrd -size h -bin -file \"your_path/output.bin\" 0x%08X %d\r\n", ResultBuffer, OutSizeHWords/2);
		
		xil_printf("\r\nSave the software referent image from RAM using the following XDB command:\r\n");
        xil_printf("mrd -size h -bin -file \"your_path/output_sw.bin\" 0x%08X %d\r\n", ReferentBuffer, OutSizeHWords/2);

        xil_printf("\r\nType '1' and press Enter to return to Main Menu: ");
        scanf("%d", &ready);

cleanup:
        free(DataBuffer);
        free(ResultBuffer);
        free(ReferentBuffer);
    }

    xil_printf("\r\n--- Exiting Program --- \r\n");
    return XST_SUCCESS;
}



// dynamically computes optimal scale to preserve Q1.15 precision and populates registers
static void CalculateScaleAndQuantize(float* float_coeffs, int n_coeffs, FilterParams* Params) {
    float max_abs = 0.0f;
    for (int i = 0; i < n_coeffs; i++) {
        if (fabsf(float_coeffs[i]) > max_abs) {
            max_abs = fabsf(float_coeffs[i]);
        }
    }
    
    if (max_abs == 0.0f) {
        Params->CoeffScale = 4096;
        return;
    }

    // attempt to scale max coefficient to ~0.95 in Q1.15 to avoid overflow but keep precision
    float inv_scale = 0.95f / max_abs;
    
    // hw scale is inversely proportional to inv_scale; limit it to max 15.99 for Q4.12
    float hw_scale = 1.0f / inv_scale;
    if (hw_scale > 15.99f) {
        hw_scale = 15.99f;
        inv_scale = 1.0f / hw_scale;
    }

    Params->CoeffScale = (u16)roundf(hw_scale * 4096.0f);
    
    for (int i = 0; i < n_coeffs; i++) {
        float scaled_c = float_coeffs[i] * inv_scale;
        Params->Coeffs[i] = (s16)roundf(scaled_c * 32768.0f);
    }
}

static void GenerateBoxFilter(FilterParams* Params) {
    float C[81] = {0};
    int R = Params->Radius;
    int k_size = 2 * R + 1;
    float val = 1.0f / (k_size * k_size);

    for (int r = -R; r <= R; r++) {
        for (int c = -R; c <= R; c++) {
            C[(r + R) * 9 + (c + R)] = val;
        }
    }
    CalculateScaleAndQuantize(C, 81, Params);
}

static void GenerateGaussianFilter(FilterParams* Params, float sigma) {
    float C[81] = {0};
    int R = Params->Radius;
    float sum = 0.0f;

    for (int r = -R; r <= R; r++) {
        for (int c = -R; c <= R; c++) {
            float g = expf(-(r * r + c * c) / (2.0f * sigma * sigma));
            C[(r + R) * 9 + (c + R)] = g;
            sum += g;
        }
    }
    if(sum > 0.0f) {
        for (int i = 0; i < 81; i++) C[i] /= sum;
    }
    CalculateScaleAndQuantize(C, 81, Params);
}

static void GenerateLoGFilter(FilterParams* Params, float sigma) {
    float C[81] = {0};
    int R = Params->Radius;
    int k_size = 2 * R + 1;
    float sum = 0.0f;

    for (int r = -R; r <= R; r++) {
        for (int c = -R; c <= R; c++) {
            float r2 = r * r + c * c;
            float s2 = sigma * sigma;
            // LoG function
            float g = -(1.0f / (M_PI * s2 * s2)) * (1.0f - r2 / (2.0f * s2)) * expf(-r2 / (2.0f * s2));
            C[(r + R) * 9 + (c + R)] = g;
            sum += g;
        }
    }
    // normalize so the mean is 0
    float mean = sum / (k_size * k_size);
    for (int r = -R; r <= R; r++) {
        for (int c = -R; c <= R; c++) {
            C[(r + R) * 9 + (c + R)] -= mean;
        }
    }
    CalculateScaleAndQuantize(C, 81, Params);
}

static void GenerateSharpenFilter(FilterParams* Params, float sigma, int use_gaussian, float k) {
    float Base[81] = {0};
    int R = Params->Radius;
    float sum = 0.0f;

    // first generate the base smoothing filter
    for (int r = -R; r <= R; r++) {
        for (int c = -R; c <= R; c++) {
            float val = 1.0f;
            if (use_gaussian) {
                val = expf(-(r * r + c * c) / (2.0f * sigma * sigma));
            }
            Base[(r + R) * 9 + (c + R)] = val;
            sum += val;
        }
    }
    
    float C[81] = {0};
    for (int r = -R; r <= R; r++) {
        for (int c = -R; c <= R; c++) {
            int i = (r + R) * 9 + (c + R);
            float base_val = Base[i] / sum; // normalized
            float sharpen_val = -k * base_val;
            
            if (r == 0 && c == 0) sharpen_val += (1.0f + k); // add delta(x,y)
            C[i] = sharpen_val;
        }
    }
    CalculateScaleAndQuantize(C, 81, Params);
}

static void GenerateSobelFilter(FilterParams* Params, int axis) {
    float C[81] = {0};
    int R = Params->Radius;
    float sum_pos = 0.0f;

    // sobel approximated dynamically by spatial derivative of Gaussian
    float sigma = ((float)R) / 2.0f; 
    if(sigma < 1.0f) sigma = 1.0f;
    for (int r = -R; r <= R; r++) {
        for (int c = -R; c <= R; c++) {
            float val = (axis == 0) ? (float)c : (float)r;
            val *= expf(-(r * r + c * c) / (2.0f * sigma * sigma));
            C[(r + R) * 9 + (c + R)] = val;
            if (val > 0) sum_pos += val;
        }
    }
    if (sum_pos > 0.0f) {
        for (int i = 0; i < 81; i++) C[i] /= sum_pos;
    }
    CalculateScaleAndQuantize(C, 81, Params);
}

static void GeneratePrewittFilter(FilterParams* Params, int axis) {
    float C[81] = {0};
    int R = Params->Radius;
    float sum_pos = 0.0f;

    // prewitt is pure linear derivative across specified axis with no orthogonal smoothing
    for (int r = -R; r <= R; r++) {
        for (int c = -R; c <= R; c++) {
            float val = (axis == 0) ? (float)c : (float)r;
            C[(r + R) * 9 + (c + R)] = val;
            if (val > 0) sum_pos += val;
        }
    }
    if (sum_pos > 0.0f) {
        for (int i = 0; i < 81; i++) C[i] /= sum_pos;
    }
    CalculateScaleAndQuantize(C, 81, Params);
}


static void FilterImageSW(u8* DataBuffer, u16* ResultBuffer, FilterParams Params)
{
    int R = Params.Radius;
    int K = 2 * R + 1;
    int W = Params.Img.Width;
    int H = Params.Img.Height;
    int mode = Params.Ctrl & 0x01; 
    int bypass = (Params.Ctrl & 0x02) != 0;

    int i_out = 0;

    for (int plane = 0; plane < Params.Img.NumOfPlanes; plane++){
        int plane_offset = plane * W * H;

        for (int row = R; row < H - R; row++) {
            for (int col = R; col < W - R; col++) {
                
                if (bypass) {
                    if (mode == 0) {
                        ResultBuffer[i_out++] = (u16)DataBuffer[plane_offset + row * W + col];
                    } else {
                        ResultBuffer[i_out++] = ((u16)DataBuffer[plane_offset + row * W + col]) << 7;
                    }
                    continue;
                }

                s64 total_sum = 0; 
                
                for (int kr = 0; kr < K; kr++) {
                    for (int kc = 0; kc < K; kc++) {
                        int img_r = row + kr - R;
                        int img_c = col + kc - R;
                        
                        s32 pixel = (s32)DataBuffer[plane_offset + img_r * W + img_c]; 
                        s32 coeff = (s32)(Params.Coeffs[kr * 9 + kc]); 
                        total_sum += (pixel * coeff);
                    }
                }

                s64 scaled = total_sum * (s64)((u32)Params.CoeffScale);

                if (mode == 0) { // 8-bit unsigned output
                    s64 integer_part = scaled >> 27; // shift for Q1.15 * Q4.12
                    if (integer_part < 0) ResultBuffer[i_out] = 0;
                    else if (integer_part > 255) ResultBuffer[i_out] = 255;
                    else ResultBuffer[i_out] = (u16)integer_part;
                } else { // 16-bit Q9.7 output
                    s64 integer_part = scaled >> 20; 
                    if (integer_part > 32767) {
                        ResultBuffer[i_out] = 32767;
                    } else if (integer_part < -32768) {
                        ResultBuffer[i_out] = (u16)-32768;
                    } else {
                        ResultBuffer[i_out] = (u16)(integer_part & 0xFFFF);
                    }
                }
                i_out++;
            }
        }
    }
}


static int FilterImageHW(u8* DataBuffer, u16* ResultBuffer, FilterParams Params)
{ 
    int Status;
    u32 PlaneInSize = Params.Img.Height * Params.Img.Width;

    u32 outW = Params.Img.Width - 2 * Params.Radius;
    u32 outH = Params.Img.Height - 2 * Params.Radius;
    u32 PlaneOutSizeBytes = outW * outH * sizeof(u16);

    XAxiDma_Config *AxiDmaConfigPtr = XAxiDma_LookupConfig(XPAR_XAXIDMA_0_BASEADDR);
    if (!AxiDmaConfigPtr) {
        xil_printf("  HW CONFIG ERROR: No config found for %d\r\n", XPAR_XAXIDMA_0_BASEADDR);
        return XST_FAILURE;
    }

    Status = DmaConfigure(AxiDmaConfigPtr, &AxiDma);
    if (Status != XST_SUCCESS) {
        xil_printf("  HW CONFIG ERROR: DMA configuration failed\r\n");
        return XST_FAILURE;        
    }

    for(int p = 0; p < Params.Img.NumOfPlanes; p++) {
        TxDone = 0;
        RxDone = 0;
        
        u8* tx_ptr = DataBuffer + p * PlaneInSize;
        u8* rx_ptr = (u8*)(ResultBuffer + p * outW * outH);

        Status = DmaStartTransfers(&AxiDma, tx_ptr, PlaneInSize, rx_ptr, PlaneOutSizeBytes);
        if (Status != XST_SUCCESS) {
            xil_printf("  HW CONFIG ERROR: Starting DMA transfer failed for plane %d\r\n", p);
            return XST_FAILURE;        
        }

        Status = DmaWaitTransfers(&TxDone, &RxDone, DMA_TRANSFER_TIMEOUT);
        if (Status != XST_SUCCESS) {
            xil_printf("  HW PROC ERROR: Completing DMA transfer failed for plane %d\r\n", p);
            return XST_FAILURE;        
        }
    }

    // disable TX and RX interrupts
    XDisconnectInterruptCntrl(AxiDmaConfigPtr->IntrId[0], AxiDmaConfigPtr->IntrParent);
    XDisconnectInterruptCntrl(AxiDmaConfigPtr->IntrId[1], AxiDmaConfigPtr->IntrParent);

    return XST_SUCCESS;
}

static int CheckData(u16* ResultBuffer, u16* ReferentBuffer, FilterParams Params)
{
    u32 outW = Params.Img.Width - 2 * Params.Radius;
    u32 outH = Params.Img.Height - 2 * Params.Radius;
    
    int errors = 0; 
    u32 TotalElements = outW * outH * Params.Img.NumOfPlanes;

    // invalidate RxBuffer to force read newest values from DDR
    Xil_DCacheInvalidateRange((UINTPTR)ResultBuffer, TotalElements * sizeof(u16));

    for (u32 plane = 0; plane < Params.Img.NumOfPlanes; plane++) {
        u32 offset = plane * outW * outH;
        for (u32 row = 0; row < outH; row++) {
            for (u32 col = 0; col < outW; col++) {
                u32 i = offset + row * outW + col;
                
                if (ResultBuffer[i] != ReferentBuffer[i]) {
                    if (errors < 15) {
                        xil_printf("  MISMATCH [Plane %d] at (%d,%d): HW=%d, SW=%d\r\n",
                                   plane, row, col, ResultBuffer[i], ReferentBuffer[i]);
                    }
                    errors++;
                }
            }
        }
    }

    if (errors > 0) {
        xil_printf("  Total mismatches: %d / %u\r\n", errors, TotalElements);
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

static int AccConfigure(UINTPTR BaseAddress, FilterParams Params)
{
    u16 readback;

    Xil_Out32(BaseAddress + REG_CTRL_ADDR, Params.Ctrl);
    readback = Xil_In32((UINTPTR)BaseAddress + REG_CTRL_ADDR);
    if (readback != Params.Ctrl) return XST_FAILURE;

    Xil_Out32(BaseAddress + REG_RADIUS_ADDR, Params.Radius);
    readback = Xil_In32((UINTPTR)BaseAddress + REG_RADIUS_ADDR);
    if ((readback & 0x07) != Params.Radius) return XST_FAILURE;

    Xil_Out32(BaseAddress + REG_IMG_W_ADDR, Params.Img.Width);
    readback = Xil_In32((UINTPTR)BaseAddress + REG_IMG_W_ADDR);
    if (readback != Params.Img.Width) return XST_FAILURE;

    Xil_Out32(BaseAddress + REG_IMG_H_ADDR, Params.Img.Height);
    readback = Xil_In32((UINTPTR)BaseAddress + REG_IMG_H_ADDR);
    if (readback != Params.Img.Height) return XST_FAILURE;

    Xil_Out32(BaseAddress + REG_COEFF_SCALE_ADDR, Params.CoeffScale);
    readback = Xil_In32((UINTPTR)BaseAddress + REG_COEFF_SCALE_ADDR);
    if (readback != Params.CoeffScale) return XST_FAILURE;

    for (int i = 0; i < 81; i++) {
        Xil_Out32(BaseAddress + REG_COEFF_W0_ADDR + i*4, (u16)Params.Coeffs[i]);
    }

    for (int i = 0; i < 81; i++) {
        readback = Xil_In32((UINTPTR)BaseAddress + REG_COEFF_W0_ADDR + i*4);
        if (readback != (u16)Params.Coeffs[i]) return XST_FAILURE;
    }

    return XST_SUCCESS;
}

static int DmaConfigure(XAxiDma_Config* AxiDmaConfigPtr, XAxiDma* AxiDmaPtr)
{
    int Status = XAxiDma_CfgInitialize(AxiDmaPtr, AxiDmaConfigPtr);
    if (Status != XST_SUCCESS) return XST_FAILURE;

    if (XAxiDma_HasSg(AxiDmaPtr)) return XST_FAILURE;

    Status = XSetupInterruptSystem(AxiDmaPtr, &TxIntrHandler,
                                  AxiDmaConfigPtr->IntrId[0], AxiDmaConfigPtr->IntrParent,
                                  XINTERRUPT_DEFAULT_PRIORITY);
    if (Status != XST_SUCCESS) return XST_FAILURE;

    Status = XSetupInterruptSystem(AxiDmaPtr, &RxIntrHandler,
                                   AxiDmaConfigPtr->IntrId[1], AxiDmaConfigPtr->IntrParent,
                                   XINTERRUPT_DEFAULT_PRIORITY);
    if (Status != XST_SUCCESS) return XST_FAILURE;

    XAxiDma_IntrEnable(&AxiDma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);
    XAxiDma_IntrEnable(&AxiDma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);

    return XST_SUCCESS;
}

static int DmaStartTransfers(XAxiDma* AxiDmaPtr, u8* TxBuffer, u32 TxSize, u8* RxBuffer, u32 RxSize)
{
    Xil_DCacheFlushRange((UINTPTR)TxBuffer, TxSize);
    Xil_DCacheFlushRange((UINTPTR)RxBuffer, RxSize);

    int Status = XAxiDma_SimpleTransfer(AxiDmaPtr, (UINTPTR) RxBuffer, RxSize, XAXIDMA_DEVICE_TO_DMA);
    if (Status != XST_SUCCESS) return XST_FAILURE;

    Status = XAxiDma_SimpleTransfer(&AxiDma, (UINTPTR) TxBuffer, TxSize, XAXIDMA_DMA_TO_DEVICE);
    if (Status != XST_SUCCESS) return XST_FAILURE;

    return XST_SUCCESS;
}

static int DmaWaitTransfers(volatile u32* TxFlag, volatile u32* RxFlag, u32 Timeout)
{
    int Status = Xil_WaitForEventSet(Timeout, 1, TxFlag);
    if (Status != XST_SUCCESS) return XST_FAILURE;

    Status = Xil_WaitForEventSet(Timeout, 1, RxFlag);
    if (Status != XST_SUCCESS) return XST_FAILURE;

    return XST_SUCCESS;
}

static void TxIntrHandler(void *Callback)
{
    XAxiDma *AxiDmaInst = (XAxiDma *)Callback;
    u32 IrqStatus = XAxiDma_IntrGetIrq(AxiDmaInst, XAXIDMA_DMA_TO_DEVICE);
    XAxiDma_IntrAckIrq(AxiDmaInst, IrqStatus, XAXIDMA_DMA_TO_DEVICE);
    if ((IrqStatus & XAXIDMA_IRQ_IOC_MASK)) TxDone = 1;
}

static void RxIntrHandler(void *Callback)
{
    XAxiDma *AxiDmaInst = (XAxiDma *)Callback;
    u32 IrqStatus = XAxiDma_IntrGetIrq(AxiDmaInst, XAXIDMA_DEVICE_TO_DMA);
    XAxiDma_IntrAckIrq(AxiDmaInst, IrqStatus, XAXIDMA_DEVICE_TO_DMA);
    if ((IrqStatus & XAXIDMA_IRQ_IOC_MASK)) RxDone = 1;
}