module sample_pulse_generator(clk, out);
    parameter WIDTH = 16;
    parameter SAMPLE_CNT_MAX = 65000;
    input clk;
    output out;

    wire [WIDTH-1:0] counter_output;
    wrapping_counter #(.WIDTH(WIDTH), .SAMPLE_CNT_MAX(SAMPLE_CNT_MAX)) cnt(.clk(clk), .rst(1'b0), .en(1'b1), .value(counter_output));
    assign out = counter_output === SAMPLE_CNT_MAX;

endmodule