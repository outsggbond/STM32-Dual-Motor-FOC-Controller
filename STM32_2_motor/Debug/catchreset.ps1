# Catch an intermittent reset and name its cause.
#   .\catchreset.ps1 -Seconds 60
#   .\catchreset.ps1 -Seconds 60 -WatchPort      # also count boot banners
#
# Clears RCC_CSR once, then polls it (plus uwTick) until something resets the
# board. The first nonzero RCC_CSR tells you WHICH pin/rail did it:
#   PORRSTF (bit27) = power-on / brownout  -> supply or GND contact
#   PINRSTF (bit26) = NRST pin             -> the reset line itself
#   SFTRSTF (bit28) = software reset       -> debugger
# A uwTick that goes backwards is a reset even if the flag read was missed
# between polls, so both are checked.
#
# Run it and wiggle one wire at a time; the moment the count goes up you know
# which wire it was.
# Keep this file ASCII-only (Windows PowerShell 5.1 reads .ps1 as ANSI/GBK).
param(
    [string] $PortName  = "COM4",
    [int]    $Seconds   = 60,
    [switch] $WatchPort
)

$cli  = "K:\STM32CubeIDE\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe"
$CSR  = "0x40021024"
$TICK = "0x200014B0"

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
function Clear-Flags { [void](& $cli -c port=SWD mode=hotplug -w32 $CSR 0x01000000 2>&1) }
function Describe([uint32]$csr) {
    if ($csr -eq 0) { return "no reset" }
    $n = @()
    if ($csr -band 0x08000000) { $n += "POR(brownout/power)" }
    if ($csr -band 0x04000000) { $n += "PIN(NRST)" }
    if ($csr -band 0x10000000) { $n += "SOFT" }
    if ($csr -band 0x20000000) { $n += "IWDG" }
    if ($csr -band 0x40000000) { $n += "WWDG" }
    if ($csr -band 0x80000000) { $n += "LPWR" }
    if ($n.Count -eq 0) { return ("unknown 0x{0:X8}" -f $csr) }
    return ($n -join "+")
}

$port = $null
if ($WatchPort) {
    $port = New-Object System.IO.Ports.SerialPort -ArgumentList @(
        $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
        [System.IO.Ports.StopBits]::One)
    $port.ReadTimeout  = 100
    $port.WriteTimeout = 1000
    $port.DtrEnable    = $false
    $port.RtsEnable    = $false
    try { $port.Open() } catch { Write-Output ("OPEN FAILED: " + $_.Exception.Message); exit 1 }
}

Clear-Flags
$prevTick = Read-U32 $TICK
$events   = 0
$boots    = 0
$sw       = [System.Diagnostics.Stopwatch]::StartNew()

Write-Output ("watching {0} s for a reset (csr cleared, baseline tick = {1})" -f $Seconds, $prevTick)
if ($WatchPort) { Write-Output "boot banners on the port are counted too -- wiggle one wire at a time" }
Write-Output ""

while ($sw.ElapsedMilliseconds -lt ($Seconds * 1000)) {

    if ($port) {
        try {
            $tmp = New-Object char[] 512
            $n = $port.Read($tmp, 0, $tmp.Length)
            if ($n -gt 0) {
                $s = -join $tmp[0..($n - 1)]
                $b = ([regex]::Matches($s, 'STM32_2_motor ready')).Count
                if ($b -gt 0) {
                    $boots += $b
                    Write-Output ("  [{0,6} ms] BOOT banner #{1}" -f $sw.ElapsedMilliseconds, $boots)
                }
            }
        } catch [System.TimeoutException] {
        } catch { }
    }

    $csr  = Read-U32 $CSR
    $tick = Read-U32 $TICK

    if ($csr -gt 0) {
        $events++
        Write-Output ("  [{0,6} ms] *** RESET #{1} -- cause: {2}  (RCC_CSR=0x{3:X8}, uptime was {4} ms) ***" -f `
            $sw.ElapsedMilliseconds, $events, (Describe $csr), $csr, $tick)
        Clear-Flags
        $prevTick = Read-U32 $TICK
    } elseif ($tick -lt $prevTick) {
        $events++
        Write-Output ("  [{0,6} ms] *** RESTART #{1} without a flag -- uptime {2} ms < {3} ms ***" -f `
            $sw.ElapsedMilliseconds, $events, $tick, $prevTick)
        $prevTick = $tick
    } else {
        $prevTick = $tick
    }

    Start-Sleep -Milliseconds 250
}

if ($port) { $port.Close() }
Write-Output ""
if ($events -eq 0) {
    Write-Output ("clean: {0} s with no reset and no restart." -f $Seconds)
} else {
    Write-Output ("{0} reset/restart events in {1} s." -f $events, $Seconds)
}
if ($WatchPort) { Write-Output ("boot banners seen: {0}" -f $boots) }
