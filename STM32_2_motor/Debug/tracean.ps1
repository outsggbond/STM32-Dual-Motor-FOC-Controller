# Analyse a posstep.csv trace: differentiate the angle column to get velocity and
# acceleration, and report how the rotor actually decelerates on the way to target.
#   .\tracean.ps1 -Motor 2 -Step 90
# Keep this file ASCII-only (Windows PowerShell 5.1 reads .ps1 as ANSI/GBK).
param(
    [int]    $Motor   = 2,
    [int]    $Step    = 90,
    [string] $CsvPath = "posstep.csv"
)

$rows = Import-Csv $CsvPath |
    Where-Object { $_.motor -eq "$Motor" -and $_.step_deg -eq "$Step" }

foreach ($grp in ($rows | Group-Object rep)) {

    $pts = $grp.Group | Sort-Object { [double]$_.t_ms }
    $n = $pts.Count
    if ($n -lt 4) { continue }

    $t = @(); $a = @()
    foreach ($p in $pts) { $t += [double]$p.t_ms; $a += [double]$p.deg }

    # velocity and acceleration by central difference
    $v = New-Object double[] $n
    $acc = New-Object double[] $n
    for ($i = 1; $i -lt $n; $i++) {
        $dt = ($t[$i] - $t[$i-1]) / 1000.0
        if ($dt -gt 0) { $v[$i] = ($a[$i] - $a[$i-1]) / $dt }
    }
    for ($i = 1; $i -lt $n; $i++) {
        $dt = ($t[$i] - $t[$i-1]) / 1000.0
        if ($dt -gt 0) { $acc[$i] = ($v[$i] - $v[$i-1]) / $dt }
    }

    $vPeak = 0.0; $vPeakT = 0.0
    for ($i = 0; $i -lt $n; $i++) {
        if ($v[$i] -gt $vPeak) { $vPeak = $v[$i]; $vPeakT = $t[$i] }
    }

    # first sample at or past the target, and the peak angle after that
    $iTarget = -1
    for ($i = 0; $i -lt $n; $i++) {
        if ($a[$i] -ge $Step) { $iTarget = $i; break }
    }

    $peakA = $a[0]; $peakT = $t[0]
    for ($i = 0; $i -lt $n; $i++) {
        if ($a[$i] -gt $peakA) { $peakA = $a[$i]; $peakT = $t[$i] }
    }

    # worst braking acceleration (most negative) over the whole trace
    $dec = 0.0; $decT = 0.0
    for ($i = 1; $i -lt $n; $i++) {
        if ($acc[$i] -lt $dec) { $dec = $acc[$i]; $decT = $t[$i] }
    }

    Write-Output ("rep {0}: n={1}  peakAngle={2:N2} at t={3:N0}ms  overshoot={4:N2} deg" -f `
        $grp.Name, $n, $peakA, $peakT, ($peakA - $Step))
    Write-Output ("        v_peak={0:N0} deg/s at t={1:N0}ms   max decel={2:N0} deg/s^2 at t={3:N0}ms" -f `
        $vPeak, $vPeakT, $dec, $decT)

    if ($iTarget -gt 0) {
        $dtT = ($t[$iTarget] - $t[$iTarget-1]) / 1000.0
        $dts = ($t[$iTarget] - $t[0]) / 1000.0
        Write-Output ("        speed at first crossing of {0} deg: {1:N0} deg/s" -f $Step, $v[$iTarget])
        Write-Output ("        travel left after crossing: {0:N2} deg" -f ($peakA - $Step))
        # distance the rotor still needed given its speed and the measured braking
        if ($dec -lt -1.0) {
            $need = ($v[$iTarget] * $v[$iTarget]) / (2.0 * [math]::Abs($dec))
            Write-Output ("        braking distance needed at that speed = {0:N2} deg" -f $need)
            Write-Output ("        (curve assumed amax=60 rad/s^2 -> {0:N2} deg)" -f `
                (($v[$iTarget] * $v[$iTarget]) / (2.0 * 60.0 * 57.2958)))
        }
    }
}
