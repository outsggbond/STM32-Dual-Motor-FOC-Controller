# Count boot banners over a plain serial listen -- no SWD, no DTR/RTS unless asked.
#   .\listen.ps1 -Seconds 10          # DTR=0 RTS=0
#   .\listen.ps1 -Seconds 10 -Dtr     # DTR=1 RTS=1 (what ser.ps1 / posstep.ps1 do)
#
# The firmware prints "STM32_2_motor ready" once per boot (uart_debug.c:1035), so
# the number of banners in the window IS the number of resets. Comparing the two
# DTR settings separates "the board resets itself" from "our own scripts reset it
# by pulling NRST through the CH340's DTR/RTS".
#
# Nothing is transmitted, so this cannot confuse the firmware's command parser.
# Keep this file ASCII-only (Windows PowerShell 5.1 reads .ps1 as ANSI/GBK).
param(
    [string] $PortName = "COM4",
    [int]    $Seconds  = 10,
    [switch] $Dtr
)

$port = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$port.ReadTimeout  = 100
$port.WriteTimeout = 1000
$port.DtrEnable    = [bool]$Dtr
$port.RtsEnable    = [bool]$Dtr
try { $port.Open() } catch { Write-Output "OPEN FAILED: $($_.Exception.Message)"; exit 1 }

Write-Output ("listening {0} s on {1} with DTR={2} RTS={2} (no SWD attached)" -f `
    $Seconds, $PortName, [int][bool]$Dtr)

$sw   = [System.Diagnostics.Stopwatch]::StartNew()
$buf  = New-Object System.Text.StringBuilder
$tmp  = New-Object char[] 512
$boots = New-Object System.Collections.ArrayList
$last  = ""

while ($sw.ElapsedMilliseconds -lt ($Seconds * 1000)) {
    try {
        $n = $port.Read($tmp, 0, $tmp.Length)
        if ($n -gt 0) {
            [void]$buf.Append($tmp, 0, $n)
            $txt = $buf.ToString()
            # keep the trailing partial line for the next round
            $cut = $txt.LastIndexOf("`n")
            if ($cut -ge 0) {
                $whole = $txt.Substring(0, $cut)
                [void]$buf.Clear()
                [void]$buf.Append($txt.Substring($cut + 1))
                foreach ($ln in ($whole -split "`n")) {
                    $t = $ln.Trim()
                    if ($t -match 'STM32_2_motor ready') {
                        [void]$boots.Add($sw.ElapsedMilliseconds)
                        Write-Output ("  [{0,6} ms] BOOT" -f $sw.ElapsedMilliseconds)
                    }
                }
            }
        }
    } catch [System.TimeoutException] {
    } catch { break }
}
$port.Close()

Write-Output ""
if ($boots.Count -eq 0) {
    Write-Output "0 boot banners -- the board did NOT reset during this window."
} else {
    Write-Output ("{0} boot banners in {1} s -- the board is resetting on its own." -f `
        $boots.Count, $Seconds)
    if ($boots.Count -ge 2) {
        $gaps = @()
        for ($i = 1; $i -lt $boots.Count; $i++) { $gaps += ($boots[$i] - $boots[$i-1]) }
        Write-Output ("  gaps between boots (ms): {0}" -f ($gaps -join ", "))
    }
}
