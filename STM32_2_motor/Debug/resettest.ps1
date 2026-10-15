# Decide what resets the board, by clearing RCC_CSR before each phase.
#   .\resettest.ps1
#
# RCC_CSR (0x40021024) latches WHY the last reset happened and only a write of
# RMVF (bit24) clears it. So: clear it, run one phase, read it back. A flag that
# reappears was caused by THAT phase and names the cause directly --
#   bit27 PORRSTF  = power-on / brownout      (supply sag)
#   bit26 PINRSTF  = NRST pin                 (external reset line)
#   bit28 SFTRSTF  = software reset           (debugger / NVIC_SystemReset)
# uwTick (0x200014B0) is half the evidence too: it only counts up, so if it does
# not advance by roughly the wall-clock gap, the board restarted.
#
# Phases: port closed / port open with DTR=RTS=0 / port open with DTR=RTS=1.
# Keep this file ASCII-only (Windows PowerShell 5.1 reads .ps1 as ANSI/GBK).
param(
    [string] $PortName = "COM4",
    [int]    $WaitMs   = 5000
)

$cli = "K:\STM32CubeIDE\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe"
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

# RMVF (bit24): writing 1 clears every reset flag. The CLI reports a spurious
# "Failed to download data" for this write; the read-back below is the truth.
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

function Run-Phase([string]$name, [string]$mode) {
    Clear-Flags
    Start-Sleep -Milliseconds 300
    $csr0 = Read-U32 $CSR
    $t0   = Read-U32 $TICK

    $port = $null
    $boots = 0
    if ($mode -ne "closed") {
        $port = New-Object System.IO.Ports.SerialPort -ArgumentList @(
            $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
            [System.IO.Ports.StopBits]::One)
        $port.ReadTimeout  = 100
        $port.WriteTimeout = 1000
        $assert = ($mode -eq "dtr1")
        $port.DtrEnable = $assert
        $port.RtsEnable = $assert
        try { $port.Open() } catch {
            Write-Output ("{0}: OPEN FAILED {1}" -f $name, $_.Exception.Message); return
        }
    }

    # wait, counting boot banners if the port is open
    $sw  = [System.Diagnostics.Stopwatch]::StartNew()
    $buf = New-Object System.Text.StringBuilder
    $tmp = New-Object char[] 512
    while ($sw.ElapsedMilliseconds -lt $WaitMs) {
        if ($port) {
            try {
                $n = $port.Read($tmp, 0, $tmp.Length)
                if ($n -gt 0) { [void]$buf.Append($tmp, 0, $n) }
            } catch [System.TimeoutException] {
            } catch { }
        } else {
            Start-Sleep -Milliseconds 200
        }
    }
    if ($port) { $port.Close() }

    if ($port) {
        $txt = $buf.ToString()
        $boots = ([regex]::Matches($txt, 'STM32_2_motor ready')).Count
    }

    $t1   = Read-U32 $TICK
    $csr1 = Read-U32 $CSR

    Write-Output ("--- {0} ---" -f $name)
    Write-Output ("    csr before = 0x{0:X8}   tick before = {1}" -f $csr0, $t0)
    Write-Output ("    tick after  = {0}   advanced {1} for a {2} ms wait" -f $t1, ($t1 - $t0), $WaitMs)
    Write-Output ("    csr after   = 0x{0:X8}  -> {1}" -f $csr1, (Describe $csr1))
    if ($port) { Write-Output ("    boot banners seen on the port = {0}" -f $boots) }
    if (($t1 - $t0) -lt $WaitMs) {
        Write-Output "    *** tick did not advance -> THE BOARD RESET DURING THIS PHASE ***"
    }
}

Write-Output ("reset-cause test on {0}, {1} ms per phase" -f $PortName, $WaitMs)
Write-Output ""
Run-Phase "phase 1: serial port CLOSED"        "closed"
Write-Output ""
Run-Phase "phase 2: port open, DTR=0 RTS=0"    "dtr0"
Write-Output ""
Run-Phase "phase 3: port open, DTR=1 RTS=1"    "dtr1"
