module tb_top_axis_uart();

    // Parameters
    localparam CLK_PERIOD = 20;       // 50 MHz => 20 ns
          
    // DUT signals
    reg clk, rst;
    reg [7:0] axis_data;
    reg axis_valid;
       
    reg axis_last;
    wire uart_tx;
    wire rx_valid;
    wire [7:0] rx_data;

   
    top_axis_uart #(.DATA_BITS(8)) dut (
        .clk(clk),
        .rst(rst),
        .axis_data(axis_data),
        .axis_valid(axis_valid),
      
        .m_axis_ready(m_axis_ready),  
        .axis_last(axis_last),
        .uart_tx(uart_tx),
        .rx_valid(rx_valid),
        .rx_data(rx_data)
    );

   
    always #(CLK_PERIOD/2) clk = ~clk;

   
    task load_char(input [7:0] char);
    begin
      @(posedge clk);
        axis_data  <= char;
        
        
    end
    
    endtask

   
    integer i;
    initial begin
        
        clk = 0;
        rst = 1;
        axis_data = 0;
        axis_valid = 0;
        
        axis_last = 0;
     
        #(10*CLK_PERIOD);
        rst = 0;
     end
     initial begin
      wait(!rst && m_axis_ready);
        // Generate stream A-Z
        for (i=0; i<26; i=i+1) begin
            load_char((65 + i)); 
            @(posedge clk);
            axis_valid <= 1;
            axis_last  <= (i == 25); // last at 'Z'
            wait(m_axis_ready);        // wait until FIFO accepts
            @(posedge clk);
            axis_valid <= 0;
            axis_last  <= 0;
        end
    repeat (5) @(posedge clk);
      
         repeat (200000) @(posedge clk);  
        $finish;
    end

  
    always @(posedge clk) begin
        if (rx_valid) begin
            $display("Time %t: Received char = %s (0x%h)", $time, rx_data, rx_data);
        end
    end

endmodule
