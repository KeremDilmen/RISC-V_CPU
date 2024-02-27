//`timescale 1 ps / 1 ps

`celldefine

module BUFG
`ifdef XIL_TIMING
#(
  parameter LOC = "UNPLACED"
)
`endif
(
  output O,

  input I
);
  
// define constants
  localparam MODULE_NAME = "BUFG";

`ifdef XIL_TIMING
  reg notifier;
`endif

// begin behavioral model

    buf B1 (O, I);

// end behavioral model

`ifndef XIL_XECLIB
`ifdef XIL_TIMING
  specify
    (I => O) = (0:0:0, 0:0:0);
    $period (negedge I, 0:0:0, notifier);
    $period (posedge I, 0:0:0, notifier);
    specparam PATHPULSE$ = 0;
  endspecify
`endif
`endif

endmodule

`endcelldefine