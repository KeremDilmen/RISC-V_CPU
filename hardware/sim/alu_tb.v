module alu_tb;
    reg [31:0] a;
    reg [31:0] b;
    reg [3:0] imm_sel;
    wire [31:0] out;

    reg [31:0] expected;
  
    alu test_alu(.a(a), .b(b), .imm_sel(imm_sel), .out(out));

    reg clk;
    initial clk = 0;
    always #1 clk = ~clk;

    task check; begin
    #2;
      $display("a: %b, b: %b, out: %b, expected: %b", a, b, out, expected);
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
    a = 0; 
    b = 1; 
    imm_sel = 4'b0111;
    expected = 0;
    check;
      
    

    $display("PASS");
    $finish();
    end
endmodule