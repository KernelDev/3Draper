#!/bin/bash
# Run all_files_test and collect results
cd /home/z/my-project
echo "=== Running all_files_test ==="
timeout 300 ./target/release/all_files_test 2>/dev/null | grep -E "^test/" > /tmp/test_results.txt
echo "=== Results ==="
cat /tmp/test_results.txt
echo "=== Summary ==="
total=$(wc -l < /tmp/test_results.txt)
good=$(grep -c "GOOD" /tmp/test_results.txt 2>/dev/null || echo 0)
bad=$(grep -c "BAD" /tmp/test_results.txt 2>/dev/null || echo 0)
echo "Total: $total, GOOD: $good, BAD: $bad"
