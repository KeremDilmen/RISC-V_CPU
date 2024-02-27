module debouncer #(
   parameter WIDTH              = 1,
   parameter SAMPLE_CNT_MAX     = 25000,
   parameter PULSE_CNT_MAX      = 150,
   parameter WRAPPING_CNT_WIDTH = $clog2(SAMPLE_CNT_MAX) + 1,
   parameter SAT_CNT_WIDTH      = $clog2(PULSE_CNT_MAX) + 1
) (
   input clk,
   input [WIDTH-1:0] glitchy_signal,
   output [WIDTH-1:0] debounced_signal
);

   wire spg_out;
   wire [WIDTH - 1 : 0] sat_cnt_enable;
   wire [WIDTH - 1 : 0] sat_cnt_reset;
   wire [WIDTH - 1: 0] sync_output;

   sample_pulse_generator #(.WIDTH(WRAPPING_CNT_WIDTH), .SAMPLE_CNT_MAX(SAMPLE_CNT_MAX)) spg(.clk(clk), .out(spg_out));

   genvar i;
  
   generate
   for (i = 0; i < WIDTH; i = i + 1) begin
       assign sat_cnt_enable[i] = spg_out & glitchy_signal[i];
       assign sat_cnt_reset[i] = ~glitchy_signal[i];
       saturating_counter #(.WIDTH(SAT_CNT_WIDTH), .PULSE_CNT_MAX(PULSE_CNT_MAX)) sc(.clk(clk), .out(debounced_signal[i]), .en(sat_cnt_enable[i]), .rst(sat_cnt_reset[i]));
   end
   endgenerate


endmodule