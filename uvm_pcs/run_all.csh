#!/usr/bin/csh -f
# =============================================================
# run_all.csh — Run ALL tests + generate coverage report
# IEEE 802.3 PCS TX UVM Testbench  |  EE 273 S'26
#
# USAGE:
#   ./run_all.csh |& tee run_all.log
# =============================================================

# ---- Source environment (gives us vcs + urg in PATH) --------
source /apps/design_environment.csh
setenv UVM_HOME /home/morris/uvm-1.2

# ---- Make sv_uvm executable (fixes Permission denied) -------
chmod +x sv_uvm
chmod +x run_all.csh

# ---- Auto-detect testbench filename -------------------------
# Handles: uvm_tb.sv  OR  uvm_tb  OR  testbench.sv
if (-f uvm_tb.sv) then
    set TB = uvm_tb.sv
else if (-f uvm_tb) then
    echo "Copying uvm_tb -> uvm_tb.sv"
    cp uvm_tb uvm_tb.sv
    set TB = uvm_tb.sv
else if (-f testbench.sv) then
    set TB = testbench.sv
else
    echo "ERROR: No testbench found."
    echo "       Tried: uvm_tb.sv, uvm_tb, testbench.sv"
    echo "       Files in this directory:"
    ls -la
    exit 1
endif

# ---- Check DUT exists ---------------------------------------
if (! -f DUTS26_0.sv) then
    echo "ERROR: DUTS26_0.sv not found"
    echo "       Files here: `ls`"
    exit 1
endif

echo ""
echo "========================================================"
echo "  DUT : DUTS26_0.sv"
echo "  TB  : $TB"
echo "  Dir : `pwd`"
echo "========================================================"

# ---- Clean previous results ---------------------------------
echo "=== Cleaning old results ==="
rm -rf coverage.vdb coverage_report compile.log sim_*.log

# ---- Test list ----------------------------------------------
set TESTS = ( \
    pcs_smoke_test \
    pcs_walk1_test \
    pcs_zeros_test \
    pcs_diag2b_zeros_test \
    pcs_diag2b_ones_test \
    pcs_diag_escalate_test \
    pcs_altbit_test \
    pcs_rand_test \
    pcs_b2b_test \
    pcs_stress_test \
    pcs_exhaustive_test \
)

# ---- Run each test ------------------------------------------
set pass_count = 0
set fail_count = 0

foreach test ($TESTS)
    echo ""
    echo "========================================================"
    echo "  RUNNING: $test"
    echo "========================================================"

    ./sv_uvm DUTS26_0.sv $TB +UVM_TESTNAME=${test}

    if ($status != 0) then
        echo "  STATUS: FAILED -- $test"
        @ fail_count++
    else
        echo "  STATUS: PASSED -- $test"
        @ pass_count++
    endif
end

# ---- Generate coverage report -------------------------------
echo ""
echo "========================================================"
echo "  Generating Coverage Report"
echo "========================================================"

which urg >& /dev/null
if ($status == 0) then
    urg -dir coverage.vdb \
        -format both \
        -report coverage_report \
        -log urg.log
    if ($status == 0) then
        echo "  HTML report: coverage_report/dashboard.html"
        echo "  Text report: coverage_report/hierarchy.txt"
    else
        echo "  WARNING: urg ran but failed -- check urg.log"
    endif
else
    echo "  WARNING: urg not in PATH."
    echo "  Run manually after sourcing environment:"
    echo "    source /apps/design_environment.csh"
    echo "    urg -dir coverage.vdb -format both -report coverage_report"
endif

# ---- Final summary ------------------------------------------
echo ""
echo "========================================================"
echo "  FINAL SUMMARY"
echo "========================================================"
echo "  Passed: $pass_count / `echo $TESTS | wc -w | tr -d ' '` tests"
echo "  Failed: $fail_count / `echo $TESTS | wc -w | tr -d ' '` tests"
echo ""
echo "  --- Scoreboard per test ---"
foreach test ($TESTS)
    if (-f sim_${test}.log) then
        echo ""
        printf "  [%-30s]  " $test
        grep -E "FAILURES|TEST PASSED" sim_${test}.log | tail -1
    endif
end
echo ""
echo "  --- Coverage per test ---"
foreach test ($TESTS)
    if (-f sim_${test}.log) then
        echo ""
        echo "  [$test]"
        grep -E "stream_phase|pam5_sym|csr_patterns|sdd_pattern|esd_pattern|phase_trans" \
             sim_${test}.log | sed 's/^/    /'
    endif
end
echo ""
echo "========================================================"
echo "  Files:"
echo "    sim_*.log           per-test logs"
echo "    coverage.vdb/       VCS coverage database"
echo "    coverage_report/    HTML + text coverage report"
echo ""
echo "  Open in browser:"
echo "    firefox coverage_report/dashboard.html &"
echo "========================================================"
