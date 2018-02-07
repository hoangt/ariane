// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba, ETH Zurich
// Date: 19.03.2017
// Description: Ariane Top-level module

import ariane_pkg::*;

module ariane_wrapped #(
        parameter logic [63:0] CACHE_START_ADDR  = 64'h8000_0000, // address on which to decide whether the request is cache-able or not
        parameter int unsigned AXI_ID_WIDTH      = 10,
        parameter int unsigned AXI_USER_WIDTH    = 1,
        parameter int unsigned AXI_ADDRESS_WIDTH = 64,
        parameter int unsigned AXI_DATA_WIDTH    = 64
    )(
        input  logic                           clk_i,
        input  logic                           rst_ni,
        input  logic                           test_en_i,     // enable all clock gates for testing
        // CPU Control Signals
        input  logic                           fetch_enable_i,
        // Core ID, Cluster ID and boot address are considered more or less static
        input  logic [63:0]                    boot_addr_i,
        input  logic [ 3:0]                    core_id_i,
        input  logic [ 5:0]                    cluster_id_i,
        input  logic                           flush_req_i,
        output logic                           flushing_o,
        // Interrupt inputs
        input  logic [1:0]                     irq_i,        // level sensitive IR lines, mip & sip
        input  logic                           ipi_i,        // inter-processor interrupts
        output logic                           sec_lvl_o,    // current privilege level oot
        // Timer facilities
        input  logic [63:0]                    time_i,        // global time (most probably coming from an RTC)
        input  logic                           time_irq_i,    // timer interrupt in
        // Debug Interface
        input  logic                           debug_req_i,
        output logic                           debug_gnt_o,
        output logic                           debug_rvalid_o,
        input  logic [15:0]                    debug_addr_i,
        input  logic                           debug_we_i,
        input  logic [63:0]                    debug_wdata_i,
        output logic [63:0]                    debug_rdata_o,
        output logic                           debug_halted_o,
        input  logic                           debug_halt_i,
        input  logic                           debug_resume_i,
        
        input logic [15:0] i_dip,
        output logic [15:0] o_led
    );

    localparam int unsigned AXI_NUMBYTES = AXI_DATA_WIDTH/8;

    logic        flush_dcache_ack, flush_dcache;
    logic        flush_dcache_d, flush_dcache_q;

    AXI_BUS #(
        .AXI_ADDR_WIDTH ( 64             ),
        .AXI_DATA_WIDTH ( 64             ),
        .AXI_ID_WIDTH   ( AXI_ID_WIDTH   ),
        .AXI_USER_WIDTH ( AXI_USER_WIDTH )
    ) data_if();

    AXI_BUS #(
        .AXI_ADDR_WIDTH ( 64             ),
        .AXI_DATA_WIDTH ( 64             ),
        .AXI_ID_WIDTH   ( AXI_ID_WIDTH   ),
        .AXI_USER_WIDTH ( AXI_USER_WIDTH )
    ) bypass_if();

    AXI_BUS #(
        .AXI_ADDR_WIDTH ( 64             ),
        .AXI_DATA_WIDTH ( 64             ),
        .AXI_ID_WIDTH   ( AXI_ID_WIDTH   ),
        .AXI_USER_WIDTH ( AXI_USER_WIDTH )
    ) instr_if();

    ariane #(
        .CACHE_START_ADDR ( CACHE_START_ADDR ),
        .AXI_ID_WIDTH     ( 10               ),
        .AXI_USER_WIDTH   ( 1                )
    ) i_ariane (
        .*,
        .flush_dcache_i         ( flush_dcache         ),
        .flush_dcache_ack_o     ( flush_dcache_ack     ),
        .data_if                ( data_if              ),
        .bypass_if              ( bypass_if            ),
        .instr_if               ( instr_if             )
    );

    core2mem i_core2mem (
        .instr_if               ( instr_if             ),
        .bypass_if              ( bypass_if            ),
        .data_if                ( data_if              ),
        .*
    );

    assign flush_dcache = flush_dcache_q;
    assign flushing_o = flush_dcache_q;

    // direct store interface
    always_ff @(posedge clk_i or negedge rst_ni) begin

        automatic logic [63:0] store_address;

        if (~rst_ni) begin
            flush_dcache_q  <= 1'b0;
        end else begin
            // got acknowledge from dcache - release the flush signal
            if (flush_dcache_ack)
                flush_dcache_q <= 1'b0;

            if (flush_req_i) begin
                flush_dcache_q <= 1'b1;
            end
        end
    end

endmodule


module core2mem #(
    parameter int unsigned AXI_ID_WIDTH      = 10,
    parameter int unsigned AXI_USER_WIDTH    = 1,
    parameter int unsigned AXI_ADDRESS_WIDTH = 64,
    parameter int unsigned AXI_DATA_WIDTH    = 64
)(
    input logic         clk_i,    // Clock
    input logic         rst_ni,  // Asynchronous reset active low
    input logic [15:0] i_dip,
    output logic [15:0] o_led,
    AXI_BUS.Slave       bypass_if,
    AXI_BUS.Slave       data_if,
    AXI_BUS.Slave       instr_if
);

    logic        instr_req,     bypass_req,     data_req;
    logic [63:0] instr_address, bypass_address, data_address;
    logic        instr_we,      bypass_we,      data_we;
    logic [7:0]  instr_be,      bypass_be,      data_be;
    logic [63:0] instr_wdata,   bypass_wdata,   data_wdata;
    logic [63:0] instr_rdata,   bypass_rdata,   data_rdata;

    axi2mem #(
        .AXI_ID_WIDTH   ( AXI_ID_WIDTH      ),
        .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH ),
        .AXI_DATA_WIDTH ( AXI_DATA_WIDTH    ),
        .AXI_USER_WIDTH ( AXI_USER_WIDTH    )
    ) i_bypass (
        .clk_i  ( clk_i          ),
        .rst_ni ( rst_ni         ),
        .slave  ( bypass_if      ),
        .req_o  ( bypass_req     ),
        .we_o   ( bypass_we      ),
        .addr_o ( bypass_address ),
        .be_o   ( bypass_be      ),
        .data_o ( bypass_wdata   ),
        .data_i ( bypass_rdata   )
    );
    
    always_ff @(posedge clk_i or negedge rst_ni) begin
       if (~rst_ni) begin
            bypass_rdata <= '0;
        end else begin
            if (bypass_req & bypass_we)
                o_led <= bypass_wdata;
            else if (bypass_req)
                bypass_rdata <= i_dip;
        end
    end

    axi2mem #(
        .AXI_ID_WIDTH   ( AXI_ID_WIDTH      ),
        .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH ),
        .AXI_DATA_WIDTH ( AXI_DATA_WIDTH    ),
        .AXI_USER_WIDTH ( AXI_USER_WIDTH    )
    ) i_data (
        .clk_i  ( clk_i        ),
        .rst_ni ( rst_ni       ),
        .slave  ( data_if      ),
        .req_o  ( data_req     ),
        .we_o   ( data_we      ),
        .addr_o ( data_address ),
        .be_o   ( data_be      ),
        .data_o ( data_wdata   ),
        .data_i ( data_rdata   )
    );

sram #(.DATA_WIDTH(64), .NUM_WORDS(1024)) data_ram(
   .clk_i(clk_i),
   .req_i(data_req),
   .we_i(data_we),
   .addr_i(data_address),
   .wdata_i(data_wdata),
   .be_i(data_be),
   .rdata_o(data_rdata)
);

    axi2mem #(
        .AXI_ID_WIDTH   ( AXI_ID_WIDTH      ),
        .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH ),
        .AXI_DATA_WIDTH ( AXI_DATA_WIDTH    ),
        .AXI_USER_WIDTH ( AXI_USER_WIDTH    )
    ) i_instr (
        .clk_i  ( clk_i         ),
        .rst_ni ( rst_ni        ),
        .slave  ( instr_if      ),
        .req_o  ( instr_req     ),
        .we_o   ( instr_we      ),
        .addr_o ( instr_address ),
        .be_o   ( instr_be      ),
        .data_o ( instr_wdata   ),
        .data_i ( instr_rdata   )
    );

sram #(.DATA_WIDTH(64), .NUM_WORDS(1024)) prog_ram(
       .clk_i(clk_i),
       .req_i(instr_req),
       .we_i(instr_we),
       .addr_i(instr_address),
       .wdata_i(instr_wdata),
       .be_i(instr_be),
       .rdata_o(instr_rdata)
    );

endmodule

