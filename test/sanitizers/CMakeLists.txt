# Sanitizer canaries (see sanitizer_canary.cpp). Each test passes only if the
# sanitizer report appears in the output; the exit code is ignored on purpose.
add_executable(sanitizer_canary sanitizer_canary.cpp)
find_package(Threads REQUIRED)
target_link_libraries(sanitizer_canary PRIVATE Threads::Threads)

function(wmm_add_canary mode expected_regex)
  add_test(NAME sanitizer_canary.${mode} COMMAND sanitizer_canary ${mode})
  set_tests_properties(sanitizer_canary.${mode} PROPERTIES
    PASS_REGULAR_EXPRESSION "${expected_regex}"
    LABELS "sanitizer-canary"
    TIMEOUT 30
    # Exit with an error code instead of SIGABRT: CTest fails aborted processes
    # even when PASS_REGULAR_EXPRESSION matches.
    ENVIRONMENT "ASAN_OPTIONS=detect_leaks=1:abort_on_error=0")
endfunction()

if(WMM_SANITIZER STREQUAL "address")
  wmm_add_canary(heap-overflow "AddressSanitizer: heap-buffer-overflow")
  wmm_add_canary(leak "LeakSanitizer: detected memory leaks")
elseif(WMM_SANITIZER STREQUAL "undefined")
  wmm_add_canary(signed-overflow "runtime error: signed integer overflow")
elseif(WMM_SANITIZER STREQUAL "thread")
  wmm_add_canary(data-race "ThreadSanitizer: data race")
endif()
