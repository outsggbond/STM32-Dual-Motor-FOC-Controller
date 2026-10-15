# Read the firmware's g_diag block over SWD and decode it.
#   .\diag.ps1
# Encoder-health probe. Per motor it records the HAL return code of the AS5600
# read, the chip's STATUS/AGC registers, and the I2C SR1/SR2 flags after the read.
# A nonzero read_status means the I2C transfer itself failed, which would leave
# mechanical_angle stale or garbage and send the position loop chasing a value
# that is not where the rotor actually is.
# Keep this file ASCII-only (Windows PowerShell 5.1 reads .ps1 as ANSI/GBK).
param(
    [string] $Addr = "0x200005B0",
    [int]    $Len  = 128
)

$cli = "K:\STM32CubeIDE\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe"

# This CLI version dumps -r32 to stdout as lines "0xADDR : W0 W1 W2 W3" and
# rejects a filename argument. Read it live so the value is a snapshot of a
# running target (mode=hotplug does not reset the MCU).
$out = & $cli -c port=SWD mode=hotplug -r32 $Addr $Len 2>&1

$words = @()
foreach ($line in $out) {
    if ($line -match '^\s*0x[0-9A-Fa-f]{8}\s*:') {
        $body = ($line -split ':', 2)[1]
        foreach ($tok in ($body -split '\s+')) {
            if ($tok -match '^[0-9A-Fa-f]{8}$') { $words += [Convert]::ToUInt32($tok, 16) }
        }
    }
}
if ($words.Count -lt 32) {
    Write-Output ("SHORT READ: got {0} words" -f $words.Count)
    $out | Select-Object -First 30
    exit 1
}

function B([int]$w, [int]$n) { return (($words[$w] -shr ($n * 8)) -band 0xFF) }
function F([int]$w)         { return [BitConverter]::ToSingle([BitConverter]::GetBytes($words[$w]), 0) }

# word index -> byte offset 16 = status_reg[2] agc[2] ; 20 = raw_angle[2] ;
# 24/28 = i2c_sr1[2] ; 32/36 = i2c_sr2[2] ; 48/52 = mech_angle[2] ;
# 96 = state[2] control_mode[2] ; 100 = encoder_ok[2]
Write-Output ("magic=0x{0:X8}  loop_count={1}" -f $words[0], $words[1])
if (($words[0] -band 0xFFFF0000) -ne 0xA5A50000) { Write-Output "WARN: magic mismatch" }

$hal = @("HAL_OK", "HAL_ERROR", "HAL_BUSY", "HAL_TIMEOUT")
foreach ($m in 0, 1) {
    $rs     = $words[2 + $m]
    $st     = B 4 $m
    $agc    = B 4 (2 + $m)
    $raw    = if ($m -eq 0) { $words[5] -band 0xFFFF } else { ($words[5] -shr 16) -band 0xFFFF }
    $sr1    = $words[6 + $m]
    $sr2    = $words[8 + $m]
    $mech   = F (12 + $m)
    $vel    = F (14 + $m)
    $volt   = F (16 + $m)
    $stt    = B 24 $m
    $mode   = B 24 (2 + $m)
    $enc    = B 25 $m

    $rsName = "?"
    if ($rs -lt 4) { $rsName = $hal[$rs] }

    Write-Output ("M{0}: hal={1}({2})  st=0x{3:X2} agc=0x{4:X2} raw={5,5}  i2c_sr1=0x{6:X8} sr2=0x{7:X8}" -f `
        ($m + 1), $rs, $rsName, $st, $agc, $raw, $sr1, $sr2)
    Write-Output ("     mech={0,11:N6} rad  vel={1,9:N3}  volt={2,8:N3}  state={3} mode={4} enc_ok={5}" -f `
        $mech, $vel, $volt, $stt, $mode, $enc)
}
