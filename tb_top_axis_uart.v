module tb_top_axis_uart();

    // Parameters
    localparam CLK_PERIOD = 20;       // 50 MHz => 20 ns
           // stream A-Z

    // DUT signals
    reg clk, rst;
    reg [7:0] axis_data;
    reg axis_valid;
        // enough for 26
    reg axis_last;
    wire uart_tx;
    wire rx_valid;
    wire [7:0] rx_data;

    // DUT instantiation
    top_axis_uart #(.DATA_BITS(8)) dut (
        .clk(clk),
        .rst(rst),
        .axis_data(axis_data),
        .axis_valid(axis_valid),
      
        .m_axis_ready(m_axis_ready),   // only lower 4 bits used by your DUT
        .axis_last(axis_last),
        .uart_tx(uart_tx),
        .rx_valid(rx_valid),
        .rx_data(rx_data)
    );

    // Clock generation
    always #(CLK_PERIOD/2) clk = ~clk;

    // Task to load one character
    task load_char(input [7:0] char);
    begin
      @(posedge clk);
        axis_data  <= char;
        
        
    end
    
    endtask

    // Stimulus
    integer i;
    initial begin
        // Initialize
        clk = 0;
        rst = 1;
        axis_data = 0;
        axis_valid = 0;
        
        axis_last = 0;
     

        // Release reset
        #(10*CLK_PERIOD);
        rst = 0;
     end
     initial begin
      wait(!rst && m_axis_ready);
        // Generate stream A-Z
        for (i=0; i<26; i=i+1) begin
            load_char((65 + i)); // 8'h41 = "A"
            @(posedge clk);
            axis_valid <= 1;
            axis_last  <= (i == 25); // last at 'Z'
            wait(m_axis_ready);        // wait until FIFO accepts
            @(posedge clk);
            axis_valid <= 0;
            axis_last  <= 0;
        end
    repeat (5) @(posedge clk);
        // Wait for UART Rx to capture all characters
         repeat (200000) @(posedge clk);  // ~4 ms for 26 chars at 115200 baud
        $finish;
    end

    // Monitor Rx output
    always @(posedge clk) begin
        if (rx_valid) begin
            $display("Time %t: Received char = %s (0x%h)", $time, rx_data, rx_data);
        end
    end

endmodule
