# Is the MCU clock tree actually running at the frequency the firmware assumes?
#   .\clkchk.ps1 -Seconds 30
#
# HAL configures SysTick as SystemCoreClock/1000, so uwTick advances 1 per
# millisecond ONLY IF the real core clock equals the SystemCoreClock constant the
# firmware was built with. A wrong HSE value, a failed HSE->PLL switch falling back
# to HSI (8 MHz instead of 72), or a bad PLL multiplier all show up here as a ratio
# far from 1.000 -- and nothing else we measure would reveal it, because every
# software timer (CONTROL_DT, UART_TELEMETRY_MS, stall timeouts) scales with it.
#
# Wall time is sampled around each SWD read and the midpoint is used, because
# spawning STM32_Programmer_CLI costs a few hundred ms and we do not want that
# lumped into the interval.
#
# ASCII-only (Windows PowerShell 5.1 reads .ps1 as ANSI/GBK).
param(
    [int] $Seconds = 30
)

$cli     = "K:\STM32CubeIDE\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe"
$UW_TICK = "0x200014B0"

function Read-U32([string]$addr) {
    $out = & $cli -c port=SWD mode=hotplug -r32 $addr 4 2>&1
    foreach ($line in $out) {
        if ($line -match '^\s*0x[0-9A-Fa-f]{8}\s*:') {
            $body = ($line -split ':', 2)[1]
            foreach ($tok in ($body -split '\s+')) {
                if ($tok -match '^[0-9A-Fa-f]{8}$') { return [Convert]::ToUInt32($tok, 16) }
            }
        }
    }
    return -1
}

$sw = [System.Diagnostics.Stopwatch]::StartNew()

$t0 = $sw.ElapsedMilliseconds; $a = Read-U32 $UW_TICK; $t1 = $sw.ElapsedMilliseconds
if ($a -lt 0) { Write-Output "SWD READ FAILED"; exit 1 }
$wa = ($t0 + $t1) / 2.0
Write-Output ("A  uwTick = {0,10}   (wall {1} ms)" -f $a, [int]$wa)

Start-Sleep -Seconds $Seconds

$t2 = $sw.ElapsedMilliseconds; $b = Read-U32 $UW_TICK; $t3 = $sw.ElapsedMilliseconds
if ($b -lt 0) { Write-Output "SWD READ FAILED"; exit 1 }
$wb = ($t2 + $t3) / 2.0
Write-Output ("B  uwTick = {0,10}   (wall {1} ms)" -f $b, [int]$wb)

$dTick = [double]($b - $a)
$dWall = $wb - $wa

if ($b -lt $a) {
    Write-Output "!! uwTick went BACKWARDS -- the board reset during the window."
    exit 1
}
if ($dWall -le 0) { Write-Output "bad timing window"; exit 1 }

$ratio = $dTick / $dWall
$err   = ($ratio - 1.0) * 100.0

Write-Output ""
Write-Output ("  uwTick advanced   {0:N0} ticks" -f $dTick)
Write-Output ("  wall clock        {0:N0} ms" -f $dWall)
Write-Output ("  ratio             {0:N5}   ({1:N2} % error)" -f $ratio, $err)
Write-Output ""

if ([math]::Abs($err) -lt 1.0) {
    Write-Output "  OK: the 1 ms tick is real. Clock tree matches the built SystemCoreClock."
} elseif ([math]::Abs($err) -lt 5.0) {
    Write-Output "  MARGINAL: within a few percent. Suspect HSI-vs-HSE trim, not a structural fault."
} else {
    Write-Output ("  WRONG: implied real clock = {0:N2} x assumed. Every software timer" -f $ratio)
    Write-Output "         (CONTROL_DT, telemetry, stall timeout) is off by that factor."
}
