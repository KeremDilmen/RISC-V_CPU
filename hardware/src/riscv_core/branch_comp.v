module branch_comp(input signed [31:0] a, input signed [31:0] b, output br_eq, output br_lt, input [31:0] inst);
  wire br_un;
  wire is_branch;
  assign is_branch = inst[6:0] === 7'b1100011;
  assign br_un = inst[13];
  assign br_eq = is_branch === 1'b0 ? 1'b0 : br_un === 1'b1 ? $signed(a) === $signed(b) : $unsigned(a) === $unsigned(b);
  assign br_lt = is_branch === 1'b0 ? 1'b0 : br_un === 1'b0 ? $signed(a) < $signed(b) : $unsigned(a) < $unsigned(b);
endmodule 