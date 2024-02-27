module synchronizer #(parameter WIDTH = 1) (
   input [WIDTH-1:0] async_signal,
   input clk,
   output [WIDTH-1:0] sync_signal
);

   wire [WIDTH -1 :0] a;

   REGISTER #(WIDTH) r1(.q(a), .d(async_signal), .clk(clk));
   REGISTER #(WIDTH) r2(.q(sync_signal), .d(a), .clk(clk));


endmodule