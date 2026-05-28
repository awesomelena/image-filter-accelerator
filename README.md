# FPGA Image Filter Accelerator

Hardware-accelerated spatial image filtering on the **Pynq-Z2** board (Zynq-7000 SoC), achieving up to **357× speedup** over software-only processing.

Developed as a university project for the course *Digitalni VLSI Sistemi* (Digital VLSI Systems), 2025/26.

---

## Overview

This project implements a configurable **linear spatial image filter** in hardware (VHDL), supporting arbitrary filter kernels up to **9×9** in size. The hardware accelerator (`acc_image_filter`) is connected to the ARM Cortex-A9 processor via AXI4 interfaces and a DMA controller, enabling high-throughput pixel-streaming processing.

The system supports:
- Filter radii from 0 to 4 (kernel sizes: 1×1, 3×3, 5×5, 7×7, 9×9)
- Box blur, Gaussian blur, LoG (edge detection), sharpening filters, Sobel and Prewitt filters
- 8-bit unsigned output or 16-bit signed fixed-point output
- Runtime reconfiguration — new images and filter parameters can be applied without restarting
- Software/hardware result verification (`CheckData`)
- Execution time measurement via AXI Timer

---

## Repository Structure

```
dvs25-image-filter-accelerator/
├── hardware/
│   └── src/
│       ├── acc_image_filter.vhd     # Top-level accelerator module
│       ├── ALU_filter.vhd           # Filter arithmetic unit (MAC array)
│       ├── my_types_PK.vhd          # Shared type definitions package
│       ├── RAM_definitions_PK.vhd   # RAM/BRAM-related definitions package
│       └── RAM_line_buffer.vhd      # Line buffer (BRAM-based row storage)
├── block_design/
│   └── image_filter_design.bd       # Vivado block design (DMA + Timer + accelerator)
├── software/
│   └── main.c                       # Bare-metal C application (Vitis/SDK)
├── scripts/
│   └── image_filter.ipynb           # Python notebook: image prep, result display
└── README.md
```

---

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│                    Zynq-7000 PS                      │
│              ARM Cortex-A9 (main.c)                  │
└────────────────────┬────────────────────────────────┘
                     │ AXI4
          ┌──────────┴──────────┐
          │                     │
   ┌──────▼──────┐      ┌───────▼──────┐
   │  AXI Timer  │      │  AXI DMA     │
   └─────────────┘      └──┬───────┬───┘
                    AXI    │Stream │Stream
                    ┌──────▼─┐  ┌──▼──────┐
                    │ S_AXIS │  │ M_AXIS  │
                    │ data_in│  │data_out │
                    └────────┴──┴─────────┘
                         acc_image_filter
                      ┌──────────────────┐
                      │  S_AXI_CTRL      │ ← AXI4-Lite (registers)
                      │  Line Buffer     │ ← BRAM (2×FilterRadius rows)
                      │  ALU Filter      │ ← Shift-register MAC array
                      └──────────────────┘
```

Input pixels are streamed in raster-scan order (left→right, top→bottom) via AXI4-Stream. The accelerator maintains a **line buffer** (BRAM-backed) and a **shift-register arithmetic unit** to compute the weighted sum of each pixel's local neighborhood in a fully pipelined fashion.

---

## Hardware Modules

### `acc_image_filter.vhd`
Top-level module. Manages AXI4-Lite control registers, coordinates the line buffer and ALU, handles pixel validity tracking, and drives the output AXI4-Stream.

### `ALU_filter.vhd`
The filter arithmetic unit. Organized as `K` shift registers (one per filter row), each of length equal to the filter width. Each cycle, a new pixel column enters and the weighted sum across all `(2R+1)²` neighbors is computed in parallel using fixed-point multiply-accumulate.

### `RAM_line_buffer.vhd`
BRAM-based line buffer storing `2×FilterRadius` previously received rows. Each memory word stores all pixels from the same column across stored rows, enabling a single-cycle read of an entire pixel column for the ALU.

### `my_types_PK.vhd` / `RAM_definitions_PK.vhd`
Shared packages defining types, constants, and BRAM configuration used across modules.

---

## Accelerator Register Map

All registers are 16-bit, accessed via AXI4-Lite (`s_axi_ctrl`).

| Address | Register          | Description                                              |
|---------|-------------------|----------------------------------------------------------|
| `0x00`  | `reg_ctrl`        | Control: MODE (output format), BYPASS, BORD, BORDER_VALUE |
| `0x02`  | `reg_radius`      | Filter radius [0–4]; kernel size = 2·radius+1            |
| `0x04`  | `reg_img_w`       | Input image width (pixels)                               |
| `0x06`  | `reg_img_h`       | Input image height (pixels)                              |
| `0x08`  | `reg_coeff_scale` | Output scale factor (4.12 fixed-point, unsigned)         |
| `0x0A`  | `reg_coeff_W0`    | Filter coefficient W0 (1.15 signed fixed-point)          |
| …       | …                 | W1–W79 at consecutive even addresses                     |
| `0xAA`  | `reg_coeff_W80`   | Filter coefficient W80                                   |

Filter coefficients are 16-bit signed fixed-point with 1 integer bit and 15 fractional bits, covering the range [−1, +0.99997]. The scale factor compensates for coefficient quantization error.

---

## Filter Coefficient Encoding

Coefficients are represented as **Q1.15** signed fixed-point. To encode a real-valued coefficient `w`:

```
register_value = round(w * 2^15)
```

The scale factor (`coeff_scale`) is **UQ4.12** unsigned fixed-point:

```
scale_register = round(scale * 2^12)
```

**Example — 9×9 Box filter (all coefficients = 1/81):**
```
register_value = round((1/81) * 32768) = 405
scale_register = 4096  (scale = 1.0)
```

**Example — 9×9 Gaussian filter (σ=1, inverse scale = 4):**
```
Each coefficient = W_gauss(x,y) / sum * 4,  quantized to Q1.15
scale_register  = round((1/4) * 4096) = 1024
```

---

## Performance Results

Measured using the on-chip AXI Timer (filtering operation only), on a Pynq-Z2 board.

| Image     | Filter Type         | SW Time [µs] | HW Time [µs] | Speedup     |
|-----------|---------------------|--------------|--------------|-------------|
| lena_128  | Box (5×5)           | 42,651       | 406          | **105×**    |
| lena_128  | Gaussian (9×9)      | 120,304      | 397          | **303×**    |
| lena_512  | LoG (7×7)           | 1,315,591    | 5,935        | **222×**    |
| lena_512  | Gaussian (9×9)      | 2,122,011    | 5,939        | **357×**    |
| parrots   | Sharpening (7×7)    | 5,943,419    | 26,743       | **222×**    |

The hardware execution time is **independent of kernel size** — a direct consequence of fully parallel multiply-accumulate across all filter taps (O(1) per-pixel throughput). Software time scales as O(N²) with kernel size.

---

## Getting Started

### Prerequisites

- Xilinx Vivado 2020.x (or compatible) for hardware synthesis
- Vitis / Xilinx SDK for building the C application
- Pynq-Z2 board
- Python 3 with `numpy`, `matplotlib`, `Pillow` for the notebook

### Build & Deploy

1. **Open the block design** in Vivado: `block_design/image_filter_design.bd`
2. **Add VHDL sources** from `hardware/src/` to the project
3. **Run synthesis, implementation, and generate bitstream**
4. **Export hardware** (`.xsa`) and open in Vitis
5. **Build the C application** from `software/main.c`
6. **Program the board** and run via UART

### Preparing Images

Use the Jupyter notebook (`scripts/image_filter.ipynb`) to:
- Load and resize arbitrary images
- Export them as raw `uint8` binary files (`.bin`)
- Transfer to the board via `mwr` command
- Display and compare SW/HW filtered results

---

## Output Format

The accelerator output image is smaller than the input by `2×FilterRadius` in each dimension (border pixels with incomplete neighborhoods are not computed):

```
output_width  = input_width  − 2 × FilterRadius
output_height = input_height − 2 × FilterRadius
```

---

## Authors

Developed by my friend and I as part of the *Digitalni VLSI Sistemi* course, 2025/26.  

---

## License

This project is licensed under the MIT License.
