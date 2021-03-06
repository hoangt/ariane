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
// Date: 16.05.2017
// Description: Instruction Tracer Package

package instruction_tracer_pkg;
    timeunit      1ns;
    timeprecision 1ps;

    import ariane_pkg::*;
    `ifndef SYNTHESIS
//    import uvm_pkg::*;
//    `include "uvm_macros.svh"
    `include "instruction_tracer_defines.svh"
    `include "instruction_trace_item.svh"
    `include "exception_trace_item.svh"
    `include "instruction_tracer.svh"
    `endif
endpackage
