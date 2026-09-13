`timescale 1ns / 1ps
//======================================================================
//  stage2_tb.v   --   testbench for Stage 2 of async_fifo_cdc
//
//  Compile together with stage2_design.v
//
//  Three parts:
//
//    PART 1  Exhaustive test of bin2gray and gray2bin, plus a check
//            that consecutive Gray codes differ in exactly one bit.
//
//    PART 2  Exact-latency test of sync_2ff: two clock cycles, no more,
//            no less.
//
//    PART 3  The demonstration.  A 5-bit counter runs in clock domain A
//            and is observed from clock domain B.  The same counter is
//            sent across twice: once as raw binary, once Gray-coded.
//            Real wires do not all arrive at the same instant, so each
//            bit is given a slightly different delay.  We then count how
//            often each path produces a value the counter NEVER HELD.
//
//            Expected outcome: the binary path produces impossible
//            values, the Gray path produces none.  That result is the
//            entire justification for the way the async FIFO is built.
//======================================================================

module stage2_tb;

    localparam WIDTH    = 5;
    localparam MAX_CNT  = 30;      // count 0..30, no wrap (keeps checks simple)

    integer errors;
    integer checks;

    //=================================================================
    //  PART 1 support: standalone converter instances
    //=================================================================
    reg  [WIDTH-1:0] p1_bin;
    wire [WIDTH-1:0] p1_gray;
    wire [WIDTH-1:0] p1_bin_back;

    bin2gray #(.WIDTH(WIDTH)) u_b2g_test (.bin (p1_bin),  .gray(p1_gray));
    gray2bin #(.WIDTH(WIDTH)) u_g2b_test (.gray(p1_gray), .bin (p1_bin_back));

    //=================================================================
    //  Clocks.  Deliberately unrelated periods -- 10 ns and 7 ns -- so
    //  that domain B's sampling edges drift across domain A's
    //  transitions instead of always landing in a safe spot.
    //=================================================================
    reg clk_a, clk_b, rst_n;

    initial clk_a = 1'b0;
    initial clk_b = 1'b0;
    always #5.0 clk_a = ~clk_a;      // 100 MHz
    always #3.5 clk_b = ~clk_b;      // ~143 MHz

    //=================================================================
    //  PART 2 support: a synchronizer fed from its own clock domain,
    //  so the expected latency is exactly known.
    //=================================================================
    reg  [WIDTH-1:0] p2_din;
    wire [WIDTH-1:0] p2_dout;

    sync_2ff #(.WIDTH(WIDTH)) u_sync_latency (
        .clk (clk_b), .rst_n(rst_n), .din(p2_din), .dout(p2_dout)
    );

    //=================================================================
    //  PART 3 support
    //=================================================================

    // ---- the counter, living in clock domain A ----
    reg  [WIDTH-1:0] count_a;
    reg  [WIDTH-1:0] count_a_prev;
    reg              count_run;

    always @(posedge clk_a or negedge rst_n) begin
        if (!rst_n) begin
            count_a      <= {WIDTH{1'b0}};
            count_a_prev <= {WIDTH{1'b0}};
        end else if (count_run && count_a < MAX_CNT) begin
            count_a      <= count_a + 1'b1;
            count_a_prev <= count_a;
        end
    end

    // ---- its Gray-coded twin ----
    wire [WIDTH-1:0] gray_a;
    bin2gray #(.WIDTH(WIDTH)) u_b2g_ptr (.bin(count_a), .gray(gray_a));

    // ---- the wires between the domains ----------------------------
    // Each bit gets a different delay.  This models the plain physical
    // fact that five wires between two parts of a chip are not the same
    // length, so the bits of a bus do not change at the same instant.
    // The delays are identical for both buses, so the comparison is fair.
    //----------------------------------------------------------------
    wire [WIDTH-1:0] bin_wire;
    wire [WIDTH-1:0] gray_wire;

    assign #0.10 bin_wire[0]  = count_a[0];
    assign #0.90 bin_wire[1]  = count_a[1];
    assign #0.40 bin_wire[2]  = count_a[2];
    assign #1.30 bin_wire[3]  = count_a[3];
    assign #0.60 bin_wire[4]  = count_a[4];

    assign #0.10 gray_wire[0] = gray_a[0];
    assign #0.90 gray_wire[1] = gray_a[1];
    assign #0.40 gray_wire[2] = gray_a[2];
    assign #1.30 gray_wire[3] = gray_a[3];
    assign #0.60 gray_wire[4] = gray_a[4];

    // ---- both buses crossing into domain B through synchronizers ----
    wire [WIDTH-1:0] bin_synced;
    wire [WIDTH-1:0] gray_synced;

    sync_2ff #(.WIDTH(WIDTH)) u_sync_bin (
        .clk(clk_b), .rst_n(rst_n), .din(bin_wire),  .dout(bin_synced)
    );
    sync_2ff #(.WIDTH(WIDTH)) u_sync_gray (
        .clk(clk_b), .rst_n(rst_n), .din(gray_wire), .dout(gray_synced)
    );

    // ---- convert the Gray value back once it is safely in domain B ----
    wire [WIDTH-1:0] gray_synced_bin;
    gray2bin #(.WIDTH(WIDTH)) u_g2b_ptr (.gray(gray_synced), .bin(gray_synced_bin));

    //=================================================================
    //  A local copy of the gray-to-binary rule, for use inside the
    //  monitors below.
    //=================================================================
    function [WIDTH-1:0] g2b;
        input [WIDTH-1:0] g;
        integer k;
        begin
            g2b[WIDTH-1] = g[WIDTH-1];
            for (k = WIDTH-2; k >= 0; k = k - 1)
                g2b[k] = g2b[k+1] ^ g[k];
        end
    endfunction

    //=================================================================
    //  MONITORS
    //
    //  A value appearing on a crossing wire is legal only if it is the
    //  counter's current value or its immediately previous one -- those
    //  are the only two the counter actually held.  Anything else is a
    //  number that never existed in domain A.
    //=================================================================
    reg     mon_en;
    integer bin_bus_bad;      // impossible values seen on the binary wires
    integer gray_bus_bad;     // impossible values seen on the Gray wires

    always @(bin_wire) begin
        if (mon_en && bin_wire !== count_a && bin_wire !== count_a_prev) begin
            bin_bus_bad = bin_bus_bad + 1;
            if (bin_bus_bad <= 6)
                $display("   [%6.2f ns] BINARY wires show %2d  -- counter is at %2d (was %2d)",
                          $realtime, bin_wire, count_a, count_a_prev);
        end
    end

    always @(gray_wire) begin
        if (mon_en && g2b(gray_wire) !== count_a && g2b(gray_wire) !== count_a_prev) begin
            gray_bus_bad = gray_bus_bad + 1;
            if (gray_bus_bad <= 6)
                $display("   [%6.2f ns] GRAY wires show %2d  -- counter is at %2d (was %2d)",
                          $realtime, g2b(gray_wire), count_a, count_a_prev);
        end
    end

    //=================================================================
    //  Post-synchronizer checks, sampled in domain B.
    //
    //  A synchronized counter value must satisfy two things:
    //    - it can lag, but it must never RUN AHEAD of the real counter
    //    - it must never go BACKWARDS
    //  Both follow from the counter only ever counting up.
    //=================================================================
    integer bin_sync_bad;
    integer gray_sync_bad;
    reg [WIDTH-1:0] bin_sync_last;
    reg [WIDTH-1:0] gray_sync_last;

    always @(posedge clk_b) begin
        if (mon_en) begin
            // -- binary path --
            if (bin_synced > count_a) begin
                bin_sync_bad = bin_sync_bad + 1;
                if (bin_sync_bad <= 4)
                    $display("   [%6.2f ns] BINARY synced value %2d is AHEAD of counter %2d",
                              $realtime, bin_synced, count_a);
            end else if (bin_synced < bin_sync_last) begin
                bin_sync_bad = bin_sync_bad + 1;
                if (bin_sync_bad <= 4)
                    $display("   [%6.2f ns] BINARY synced value went BACKWARDS: %2d then %2d",
                              $realtime, bin_sync_last, bin_synced);
            end
            bin_sync_last = bin_synced;

            // -- gray path --
            if (gray_synced_bin > count_a) begin
                gray_sync_bad = gray_sync_bad + 1;
                $display("   [%6.2f ns] GRAY synced value %2d is AHEAD of counter %2d",
                          $realtime, gray_synced_bin, count_a);
            end else if (gray_synced_bin < gray_sync_last) begin
                gray_sync_bad = gray_sync_bad + 1;
                $display("   [%6.2f ns] GRAY synced value went BACKWARDS: %2d then %2d",
                          $realtime, gray_sync_last, gray_synced_bin);
            end
            gray_sync_last = gray_synced_bin;
        end
    end

    //=================================================================
    //  TEST SEQUENCE
    //=================================================================
    integer i;
    integer bit_diff;
    reg [WIDTH-1:0] gray_now, gray_next;

    initial begin
        $dumpfile("stage2_tb.vcd");
        $dumpvars(0, stage2_tb);

        errors        = 0;
        checks        = 0;
        bin_bus_bad   = 0;
        gray_bus_bad  = 0;
        bin_sync_bad  = 0;
        gray_sync_bad = 0;
        bin_sync_last = 0;
        gray_sync_last= 0;
        mon_en        = 1'b0;
        count_run     = 1'b0;
        p1_bin        = 0;
        p2_din        = 0;
        rst_n         = 1'b0;

        repeat (4) @(posedge clk_a);
        @(negedge clk_a);
        rst_n = 1'b1;
        @(posedge clk_a);

        //-------------------------------------------------------------
        $display("\n================================================================");
        $display("  PART 1 : converters, all %0d values", 1<<WIDTH);
        $display("================================================================");

        for (i = 0; i < (1<<WIDTH); i = i + 1) begin
            p1_bin = i[WIDTH-1:0];
            #1;
            checks = checks + 1;
            if (p1_bin_back !== p1_bin) begin
                errors = errors + 1;
                $display("  ROUND-TRIP FAIL: bin=%0d -> gray=%b -> bin=%0d",
                          p1_bin, p1_gray, p1_bin_back);
            end
        end
        $display("  round trip bin -> gray -> bin : OK for all %0d values", 1<<WIDTH);

        // one bit changes per increment
        for (i = 0; i < (1<<WIDTH)-1; i = i + 1) begin
            gray_now  = i[WIDTH-1:0]       ^ (i[WIDTH-1:0]       >> 1);
            gray_next = (i[WIDTH-1:0]+1'b1) ^ ((i[WIDTH-1:0]+1'b1) >> 1);
            bit_diff  = 0;
            begin : count_bits
                integer b;
                for (b = 0; b < WIDTH; b = b + 1)
                    if (gray_now[b] !== gray_next[b]) bit_diff = bit_diff + 1;
            end
            checks = checks + 1;
            if (bit_diff !== 1) begin
                errors = errors + 1;
                $display("  HAMMING FAIL: %0d -> %0d changes %0d bits", i, i+1, bit_diff);
            end
        end
        $display("  consecutive Gray codes differ in exactly 1 bit : OK");

        $display("\n  a few values for reference:");
        $display("    count   binary   gray");
        for (i = 5; i <= 9; i = i + 1) begin
            p1_bin = i[WIDTH-1:0];
            #1;
            $display("     %2d     %b    %b", i, p1_bin, p1_gray);
        end
        $display("    note 7 -> 8 : binary flips 4 bits, gray flips 1");

        //-------------------------------------------------------------
        $display("\n================================================================");
        $display("  PART 2 : sync_2ff latency");
        $display("================================================================");

        @(negedge clk_b);
        p2_din = 5'd21;
        @(posedge clk_b); #1;
        checks = checks + 1;
        if (p2_dout === 5'd21) begin
            errors = errors + 1;
            $display("  FAIL: output changed after 1 cycle, should take 2");
        end
        @(posedge clk_b); #1;
        checks = checks + 1;
        if (p2_dout !== 5'd21) begin
            errors = errors + 1;
            $display("  FAIL: output is %0d after 2 cycles, expected 21", p2_dout);
        end else
            $display("  input appears at the output after exactly 2 clock cycles : OK");

        //-------------------------------------------------------------
        $display("\n================================================================");
        $display("  PART 3 : sending a counter across a clock boundary");
        $display("================================================================");
        $display("  Domain A runs at 100 MHz, domain B at ~143 MHz.");
        $display("  The 5 wires between them have delays of 0.1 to 1.3 ns,");
        $display("  so the bits of the bus do not arrive together.\n");

        @(negedge clk_a);
        mon_en    = 1'b1;
        count_run = 1'b1;

        wait (count_a == MAX_CNT);
        repeat (20) @(posedge clk_b);
        mon_en = 1'b0;

        $display("\n  ------------------------------------------------------------");
        $display("   RESULTS after counting 0 to %0d", MAX_CNT);
        $display("  ------------------------------------------------------------");
        $display("   impossible values on the BINARY wires : %0d", bin_bus_bad);
        $display("   impossible values on the GRAY   wires : %0d", gray_bus_bad);
        $display("   bad values after sync, BINARY path    : %0d", bin_sync_bad);
        $display("   bad values after sync, GRAY   path    : %0d", gray_sync_bad);
        $display("  ------------------------------------------------------------");

        checks = checks + 2;
        if (gray_bus_bad != 0) begin
            errors = errors + 1;
            $display("   UNEXPECTED: the Gray wires should never show a bad value.");
        end
        if (gray_sync_bad != 0) begin
            errors = errors + 1;
            $display("   UNEXPECTED: the Gray path should never mis-synchronize.");
        end
        if (bin_bus_bad == 0)
            $display("   (note: binary happened not to glitch in this run)");

        //-------------------------------------------------------------
        $display("\n================================================================");
        $display("   checks performed : %0d", checks);
        $display("   errors found     : %0d", errors);
        if (errors == 0)
            $display("   RESULT: *** PASS ***");
        else
            $display("   RESULT: *** FAIL ***");
        $display("================================================================\n");

        $finish;
    end

    initial begin
        #100000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
