module wrapping_counter(value, clk, rst, en);
    parameter WIDTH = 16;
    parameter SAMPLE_CNT_MAX = 16'd65000;
    output [WIDTH-1:0] value;
    input clk, rst, en;
    wire [WIDTH-1:0] next;
    wire reset; 

    REGISTER_R_CE #(WIDTH) state(.q(value), .d(next), .rst(reset), .ce(en), .clk(clk));
    assign next = value + 1;
    assign reset = rst || (value === SAMPLE_CNT_MAX) ? 1'b1 : 1'b0;

endmodule