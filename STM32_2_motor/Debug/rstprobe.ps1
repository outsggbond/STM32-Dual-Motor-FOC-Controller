# Decisive reset probe.
#   .\rstprobe.ps1
#
# Reads HAL's millisecond counter uwTick (0x200014B0) over SWD, waits, reads it
# again, then opens COM4 exactly the way ser.ps1 / posstep.ps1 do -- DTR and RTS
# both asserted -- and reads a third time.
#
# uwTick only ever counts up until the MCU resets, so a value that goes backwards
# (or comes back near zero) is proof that the board really reset, and nothing else
# produces that. That separates "the board resets" from "the trace just looks like
# it did". mode=hotplug connects without resetting the target, so the SWD reads
# themselves are passive; the third reading tells us whether opening the serial
# port resets it, which on a CH340 board happens whenever DTR/RTS are wired to
# NRST/BOOT0 (that is how Arduino-style auto-reset works).
#
# Any boot banner that arrives while the port is open is printed too -- if the
# board reboots on port-open, its banner shows up right there.
#
# Keep this file ASCII-only (Windows PowerShell 5.1 reads .ps1 as ANSI/GBK).
param(
    [string] $PortName = "COM4",
    [int]    $WaitMs   = 3000
)

$cli     = "K:\STM32CubeIDE\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe"
$UW_TICK = "0x200014B0"

# Returns the word at addr, or -1 if the read failed. This CLI dumps -r32 to
# stdout as lines "0xADDR : W0 W1 W2 W3" and rejects a filename argument.
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

$a = Read-U32 $UW_TICK
if ($a -lt 0) { Write-Output "SWD READ FAILED"; exit 1 }
Write-Output ("A  uwTick now            = {0,10}  ({1:N1} s uptime)" -f $a, ($a / 1000.0))

Start-Sleep -Milliseconds $WaitMs
$b = Read-U32 $UW_TICK
Write-Output ("B  uwTick after {0,4} ms   = {1,10}  delta={2}" -f $WaitMs, $b, ($b - $a))
if ($b -lt $a) {
    Write-Output "   *** WENT BACKWARDS -- the board reset while nothing was touching it ***"
} else {
    Write-Output ("   grows by {0} ms for a {1} ms wait (the rest is CLI connect time) -- no reset" -f ($b - $a), $WaitMs)
}

# --- now do exactly what the measurement scripts do: open the port, DTR+RTS high
$port = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$port.ReadTimeout  = 100
$port.WriteTimeout = 1000
$port.DtrEnable    = $true
$port.RtsEnable    = $true
try { $port.Open() } catch { Write-Output "OPEN FAILED: $($_.Exception.Message)"; exit 1 }

Write-Output ""
Write-Output "opened $PortName with DTR=1 RTS=1 -- listening 1500 ms for a boot banner:"
$sw  = [System.Diagnostics.Stopwatch]::StartNew()
$buf = New-Object System.Text.StringBuilder
$tmp = New-Object char[] 512
while ($sw.ElapsedMilliseconds -lt 1500) {
    try {
        $n = $port.Read($tmp, 0, $tmp.Length)
        if ($n -gt 0) { [void]$buf.Append($tmp, 0, $n) }
    } catch [System.TimeoutException] {
    } catch { break }
}
$port.Close()

$txt = $buf.ToString()
if ($txt.Length -eq 0) {
    Write-Output "   (silence -- the firmware streams only while RUNNING, so this proves nothing by itself)"
} else {
    foreach ($ln in ($txt -split "`r?`n")) {
        if ($ln.Trim().Length -gt 0) { Write-Output ("   | " + $ln.Trim()) }
    }
}

Start-Sleep -Milliseconds 300
$c = Read-U32 $UW_TICK
Write-Output ""
Write-Output ("C  uwTick after port cycle = {0,10}  delta from B = {1}" -f $c, ($c - $b))
if ($c -lt $b) {
    Write-Output "   *** RESET ON PORT OPEN -- our own scripts reboot the board ***"
} else {
    Write-Output "   no reset from the port cycle"
}
