module edge_detector #(
   parameter WIDTH = 1
)(
   input clk,
   input [WIDTH-1:0] signal_in,
   output [WIDTH-1:0] edge_detect_pulse
);
   wire [WIDTH-1:0] intermediate1;
   wire [WIDTH-1:0] intermediate2;
   genvar i;
   generate
       for (i = 0; i < WIDTH; i = i + 1) begin
           REGISTER #(WIDTH) r1(.d(signal_in[i]), .q(intermediate1[i]), .clk(clk));
           REGISTER #(WIDTH) r2(.d(intermediate1[i]), .q(intermediate2[i]), .clk(clk));
           assign edge_detect_pulse[i] = intermediate1[i] > intermediate2[i];
       end
   endgenerate


endmodule