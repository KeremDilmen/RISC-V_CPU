# RISC-V CPU Implementation

This repository contains a fully functional 5-stage pipelined RISC-V CPU designed and implemented in Verilog. The CPU supports the complete RV32I Base Instruction Set and is optimized for high efficiency and performance.

## Project Structure

### `hardware/src` Directory

#### `riscv_core` Folder
- **`alu.v`**: Arithmetic Logic Unit, responsible for performing arithmetic and logical operations.
- **`branch_comp.v`**: Handles branch comparisons and decision-making.
- **`controller.v`**: Central controller for coordinating CPU operations.
- **`cpu.v`**: The top-level CPU module integrating all core components.
- **`imm_gen.v`**: Immediate value generator for decoding instruction fields.
- **`load_formatter.v`**: Formats data during load operations.
- **`opcode.vh`**: Contains opcode definitions for instruction decoding.
- **`store_formatter.v`**: Formats data during store operations.

#### `io_circuits` Folder
- **`button_parser.v`**: Parses button inputs for control signals.
- **`debouncer.v`**: Debounces noisy input signals.
- **`edge_detector.v`**: Detects rising and falling edges of input signals.
- **`sample_pulse_generator.v`**: Generates pulse signals for sampling operations.
- **`saturating_counter.v`**: Implements a counter that saturates at maximum value.
- **`synchronizer.v`**: Synchronizes signals across clock domains.
- **`uart.v`**: Top-level UART communication module.
- **`uart_receiver.v`**: Receives serial data through UART.
- **`uart_transmitter.v`**: Transmits serial data through UART.
- **`wrapping_counter.v`**: Implements a wrapping counter for continuous counting.

### Other Important Files
- **`EECS151.v`**: Top-level module for the overall project.
- **`clk_wiz.v`**: Clock wizard module for generating clock signals.
- **`z1top.v`**: Top-level module for FPGA integration.
- **`z1top.xdc`**: Constraints file for FPGA synthesis.

## Getting Started

To run and simulate the RISC-V CPU, navigate to the `hardware` directory and use the provided `Makefile` to compile and simulate the Verilog files.

## Design Documentation

For a detailed design overview, please refer to the [design document](https://tinyurl.com/4fzwns9v).
