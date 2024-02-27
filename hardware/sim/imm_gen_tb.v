module imm_gen_tb;
  reg [31:0] inst;
  reg [2:0] imm_sel;
  wire [31:0] out;

  reg [31:0] expected;
  
  imm_gen test_imm_gen(.inst(inst), .imm_sel(imm_sel), .out(out));

    reg clk;
    initial clk = 0;
    always #1 clk = ~clk;

    task check; begin
    #2;
      $display("inst: %b, out: %b, imm_sel: %b, expected: %b", inst, imm_sel, out, expected);
    if(out !== expected) begin
        $display("FAILED");
        $finish();
    end
    end endtask
                
    initial begin
    $dumpfile("dump.vcd");
    $dumpvars;
    // set all bits to 1
    // #2;#2;#2;
      inst = {{7{1'b1}},{25{1'b0}}};
      imm_sel = 3'b010;
      expected = {{7{1'b1}},{25{1'b0}}};
    check;
      
    

    $display("PASS");
    $finish();
    end
endmodule