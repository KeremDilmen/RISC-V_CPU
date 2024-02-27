module branch_comp_tb();
  reg signed [31:0] a;
  reg signed [31:0] b;
  reg br_un; 
  wire br_eq;
  wire br_lt;

  reg expected_br_eq;
  reg expected_br_lt;
  
  branch_comp test_branch_comp(.a(a), .b(b), .br_un(br_un), .br_eq(br_eq),.br_lt(br_lt));

    reg clk;
    initial clk = 0;
    always #1 clk = ~clk;

    task check; begin
    #2;
      $display("a: %b, b: %b, br_un: %b, br_eq: %b, br_lt: %b, expected_br_eq: %b, expected_br_lt: %b", a, b, br_un, br_eq, br_lt, expected_br_eq, expected_br_lt);
      if(br_eq !== expected_br_eq || br_lt !== expected_br_lt) begin
        $display("FAILED");
        $finish();
    end
    end endtask
                
    initial begin
    $dumpfile("dump.vcd");
    $dumpvars;
    // set all bits to 1
    // #2;#2;#2;
	a = -1;
    b = 1;
    br_un = 1; 
    expected_br_eq = 0; 
    expected_br_lt = 0; 
    check;
      
    

    $display("PASS");
    $finish();
    end
endmodule