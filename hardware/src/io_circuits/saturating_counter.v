module saturating_counter(out, clk, rst, en);
    parameter WIDTH = 16;
    parameter PULSE_CNT_MAX = 65000;
    wire [WIDTH-1:0] counter_val;
    wire en_counter; 
    output out;
    input clk, rst, en;
    wire[WIDTH-1:0] next;
    wire reset;

    wrapping_counter #(WIDTH) state(.clk(clk), .value(counter_val), .rst(rst), .en(en_counter));

    assign en_counter = ~en || counter_val === PULSE_CNT_MAX ? 1'b0 : 1'b1;
    assign out = counter_val === PULSE_CNT_MAX; 

endmodule