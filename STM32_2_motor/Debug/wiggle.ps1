# Catch resets while the user physically wiggles wires.
#   .\wiggle.ps1 -Seconds 180
#
# Purpose-built for a live wiggle session, so it differs from catchreset.ps1 in two
# ways that matter here:
#   * the serial port is drained in a TIGHT loop. One boot prints ~1812 bytes (banner
#     + full HELP) and the firmware's TX ring buffer is only 3072 bytes, so anything
#     slower than continuous reading silently loses banners -- and a lost banner is a
#     lost timestamp, which is the whole point of watching while someone wiggles.
#   * every event is appended to the log file immediately, so the log can be read
#     while this is still running.
#
# RCC_CSR is polled only every -CsrEveryMs, because each SWD read is a ~1.5 s CLI
# connect and blocks the serial drain. That is fine: the reset flags are sticky, so a
# slow poll never misses a cause -- it can only miss banners, which is why the poll
# is slow and the serial read is fast.
#
# Keep this file ASCII-only (Windows PowerShell 5.1 reads .ps1 as ANSI/GBK).
param(
    [string] $PortName   = "COM4",
    [int]    $Seconds    = 180,
    [string] $LogPath    = "wiggle.log",
    [int]    $CsrEveryMs = 3000
)

$cli = "K:\STM32CubeIDE\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe"
$CSR = "0x40021024"

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
    if ($csr -lt 0) { return "read failed/garbage (target probably mid-reset)" }
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

function Log([string]$text) {
    Write-Host $text
    Add-Content -Path $LogPath -Value $text -Encoding UTF8
}

Set-Content -Path $LogPath -Value ("=== wiggle session {0} ===" -f (Get-Date -Format "yyyy-MM-dd HH:mm:ss")) -Encoding UTF8

$port = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$port.ReadTimeout  = 20
$port.WriteTimeout = 1000
$port.DtrEnable    = $false
$port.RtsEnable    = $false
try { $port.Open() } catch { Log ("OPEN FAILED: " + $_.Exception.Message); exit 1 }

Clear-Flags
Log ("listening {0} s on {1}; reset flags cleared, so the next cause is latched fresh" -f $Seconds, $PortName)
Log "wiggle ONE wire at a time and note roughly when -- the log timestamps line up with it"
Log ""

$sw       = [System.Diagnostics.Stopwatch]::StartNew()
$tmp      = New-Object char[] 1024
$tail     = ""
$boots    = 0
$events   = 0
$nextCsr  = $CsrEveryMs
$lostWarn = $false

while ($sw.ElapsedMilliseconds -lt ($Seconds * 1000)) {

    # --- fast path: drain the port -------------------------------------------
    try {
        $n = $port.Read($tmp, 0, $tmp.Length)
        if ($n -gt 0) {
            $chunk = $tail + (-join $tmp[0..($n - 1)])
            $cut = $chunk.LastIndexOf("`n")
            if ($cut -ge 0) {
                $tail = $chunk.Substring($cut + 1)
                $whole = $chunk.Substring(0, $cut)
                $b = ([regex]::Matches($whole, 'STM32_2_motor ready')).Count
                if ($b -gt 0) {
                    $boots += $b
                    for ($i = 0; $i -lt $b; $i++) {
                        Log ("[{0,7} ms] BOOT #{1}" -f $sw.ElapsedMilliseconds, $boots)
                    }
                }
            } else {
                $tail = $chunk
                if ($tail.Length -gt 4096) { $tail = $tail.Substring($tail.Length - 4096) }
            }
        }
    } catch [System.TimeoutException] {
    } catch { }

    # --- slow path: latch the reset cause ------------------------------------
    if ($sw.ElapsedMilliseconds -ge $nextCsr) {
        $nextCsr = $sw.ElapsedMilliseconds + $CsrEveryMs
        $csr = Read-U32 $CSR
        if ($csr -gt 0) {
            $events++
            Log ("[{0,7} ms] *** RESET #{1} cause: {2}  (RCC_CSR=0x{3:X8}) ***" -f `
                $sw.ElapsedMilliseconds, $events, (Describe $csr), $csr)
            Clear-Flags
        } elseif ($csr -lt 0) {
            if (-not $lostWarn) {
                Log ("[{0,7} ms] note: SWD read returned garbage -- a reset was in progress at that instant" -f $sw.ElapsedMilliseconds)
                $lostWarn = $true
            }
        } else {
            $lostWarn = $false
        }
    }
}

$port.Close()
Log ""
Log ("=== done: {0} boot banners, {1} latched reset causes in {2} s ===" -f $boots, $events, $Seconds)
