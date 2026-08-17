# Screenshot helper: brings a sim window to the front and saves a PNG.
#   powershell -File tools\grab.ps1 -Which plot -Out build\plot.png
param(
  [ValidateSet("main", "plot")] [string]$Which = "plot",
  [string]$Out = "build\shot.png",
  [int]$X = 830,
  [int]$Y = 0
)

Add-Type -AssemblyName System.Windows.Forms, System.Drawing
Add-Type @"
using System;
using System.Runtime.InteropServices;
public class Grab {
  [StructLayout(LayoutKind.Sequential)] public struct RECT { public int L,T,R,B; }
  [DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern IntPtr FindWindowW(string cls, string title);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
  [DllImport("user32.dll")] public static extern bool SetWindowPos(IntPtr h, IntPtr after, int x, int y, int w, int ht, uint flags);
  [DllImport("user32.dll")] public static extern bool BringWindowToTop(IntPtr h);
  [DllImport("user32.dll")] public static extern bool IsIconic(IntPtr h);
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int cmd);
}
"@

$cls = if ($Which -eq "main") { "DroneChaseSimWnd" } else { "DroneChasePlotWnd" }
$hwnd = [Grab]::FindWindowW($cls, $null)
if ($hwnd -eq [IntPtr]::Zero) { throw "$Which window not found (is the sim running?)" }

# A minimized window reports a -32000 rect and captures as garbage.
if ([Grab]::IsIconic($hwnd)) {
  [Grab]::ShowWindow($hwnd, 9) | Out-Null   # SW_RESTORE
  Start-Sleep -Milliseconds 600
}

# HWND_TOPMOST with SWP_NOSIZE so the window keeps its own dimensions.
[Grab]::SetWindowPos($hwnd, [IntPtr](-1), $X, $Y, 0, 0, 0x1) | Out-Null
[Grab]::BringWindowToTop($hwnd) | Out-Null
Start-Sleep -Milliseconds 1200

$r = New-Object Grab+RECT
[Grab]::GetWindowRect($hwnd, [ref]$r) | Out-Null
$w = $r.R - $r.L; $h = $r.B - $r.T
if ($w -le 0 -or $h -le 0) { throw "bad window rect ${w}x${h}" }

$bmp = New-Object System.Drawing.Bitmap $w, $h
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.CopyFromScreen($r.L, $r.T, 0, 0, $bmp.Size)
$bmp.Save((Resolve-Path -LiteralPath (Split-Path $Out -Parent)).Path + "\" + (Split-Path $Out -Leaf),
          [System.Drawing.Imaging.ImageFormat]::Png)
$g.Dispose(); $bmp.Dispose()
"saved $Out (${w}x${h})"
