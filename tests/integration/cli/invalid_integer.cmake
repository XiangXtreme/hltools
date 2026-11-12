file(MAKE_DIRECTORY "${SCRATCH}")
set(map "${SCRATCH}/input.map")
file(WRITE "${map}" "{\n\"classname\" \"worldspawn\"\n}\n")

function(expect_failure flag value expected)
    execute_process(
        COMMAND "${HLTOOLS_EXE}" compile "${map}" "${flag}" "${value}"
        RESULT_VARIABLE status OUTPUT_VARIABLE output ERROR_VARIABLE error)
    if(status EQUAL 0 OR NOT "${output}${error}" MATCHES "${expected}")
        message(FATAL_ERROR "${flag} ${value}: expected ${expected}, status ${status}\n${output}${error}")
    endif()
endfunction()

expect_failure(-threads 2147483648 "-threads expects an integer")
expect_failure(-threads -2147483649 "-threads expects an integer")
expect_failure(-threads 12garbage "-threads expects an integer")
expect_failure(-threads "" "-threads expects an integer")
expect_failure(-bounce -1 "-bounce must not be negative")
expect_failure(-lightdata 2097152 "-lightdata must be between")
expect_failure(-texdata -1 "-texdata must be between")

execute_process(
    COMMAND "${HLTOOLS_EXE}" compile "${map}" -threads
    RESULT_VARIABLE status OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(status EQUAL 0 OR NOT "${output}${error}" MATCHES "-threads requires an integer value")
    message(FATAL_ERROR "missing integer value: status ${status}\n${output}${error}")
endif()
