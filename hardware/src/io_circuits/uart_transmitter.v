module uart_transmitter #(
    parameter CLOCK_FREQ = 125_000_000,
    parameter BAUD_RATE = 115_200)
(
    input clk,
    input reset,

    input [7:0] data_in,
    input data_in_valid,
    output data_in_ready,

    output serial_out
);

    wire [9:0] tmp_data;
    assign tmp_data = {1'b1, data_in[7:0], 1'b0}; // register to hold data

    localparam  SYMBOL_EDGE_TIME    =   CLOCK_FREQ / BAUD_RATE;
    localparam  CLOCK_COUNTER_WIDTH =   $clog2(SYMBOL_EDGE_TIME);

    // bit counter for counting the bits we send out
    wire [3:0] bit_counter_value;
    wire [3:0] bit_counter_next;
    wire bit_counter_ce, bit_counter_rst;

    REGISTER_R_CE #(.N(4), .INIT(0)) bit_counter (
        .q(bit_counter_value),
        .d(bit_counter_next),
        .ce(bit_counter_ce),
        .rst(bit_counter_rst),
        .clk(clk)
    );

    wire [CLOCK_COUNTER_WIDTH-1:0] clock_counter_value;
    wire [CLOCK_COUNTER_WIDTH-1:0] clock_counter_next;
    wire clock_counter_ce, clock_counter_rst;

    // Keep track of sample time and symbol edge time
    REGISTER_R_CE #(.N(CLOCK_COUNTER_WIDTH), .INIT(0)) clock_counter (
        .q(clock_counter_value),
        .d(clock_counter_next),
        .ce(clock_counter_ce),
        .rst(clock_counter_rst),
        .clk(clk)
    );

    wire symbol_edge = (clock_counter_value == SYMBOL_EDGE_TIME - 1);
    wire done        = (bit_counter_value == 10 - 1) & symbol_edge;

    // 'start' becomes HIGH once we have the 8-bit word that we want to send (once valid is high and transmitter is ready)
    wire start;
    REGISTER_R_CE #(.N(1), .INIT(0)) start_reg (
        .q(start),
        .d(1'b1),
        .ce((data_in_ready === 1'b1) && (data_in_valid === 1'b1)),
        .rst(done),
        .clk(clk)
    );

    // register to save data when ready
    wire [9:0] data;
    REGISTER_R_CE #(.N(10), .INIT(0)) data_reg (
        .q(data),
        .d(tmp_data),
        .ce((data_in_ready === 1'b1) && (data_in_valid === 1'b1)),
        .rst(done),
        .clk(clk)
    );

    assign tx_value = reset === 1'b1 ? 1'b1 : data[bit_counter_value]; 

    assign bit_counter_next = bit_counter_value + 1;
    assign bit_counter_ce   = symbol_edge;
    assign bit_counter_rst  = done | reset;

    assign clock_counter_next = clock_counter_value + 1;
    assign clock_counter_ce   = start;
    assign clock_counter_rst  = symbol_edge | done | reset;

    assign data_in_ready = ~start; 
    assign serial_out = start === 1'b1 ? tx_value : 1'b1; 

endmodule
                                                                               