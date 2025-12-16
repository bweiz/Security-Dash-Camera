#!/usr/bin/env python3
import argparse, csv, os, signal, subprocess, time
from datetime import datetime

def sh(cmd):  # run shell command, return stdout
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, check=False)
        return (out.stdout or "").strip()
    except Exception:
        return ""

def vcgencmd(arg): return sh(["vcgencmd"] + arg.split())

def mhz(name):
    s = vcgencmd(f"measure_clock {name}") 
    try: return int(s.split("=")[1]) // 1_000_000
    except Exception: return 0

def temp_c():
    s = vcgencmd("measure_temp")  # temp=44.0'C
    try: return float(s.split("=")[1].split("'")[0])
    except Exception:
        # sysfs fallback
        for p in ("/sys/class/thermal/thermal_zone0/temp",):
            try: return int(open(p).read().strip())/1000.0
            except Exception: pass
        return 0.0

def throttled_hex():
    s = vcgencmd("get_throttled")  # throttled=0x0
    return s.split("=")[-1] if "=" in s else (s or "n/a")

def read_cpu_totals():
    with open("/proc/stat") as f:
        for line in f:
            if line.startswith("cpu "):
                vals = list(map(int, line.split()[1:])) + [0]*10
                user,nice,system,idle,iowait,irq,softirq,steal = vals[:8]
                idle_all = idle + iowait
                non_idle = user + nice + system + irq + softirq + steal
                total = idle_all + non_idle
                return total, idle_all
    return 0,0

def cpu_percent(prev=None):
    t,i = read_cpu_totals()
    if not prev: return 0.0,(t,i)
    pt,pi = prev; dt,di = t-pt, i-pi
    if dt <= 0: return 0.0,(t,i)
    return max(0.0, min(100.0, 100.0*(dt-di)/dt)), (t,i)

def loadavg():
    a,b,c,_ = open("/proc/loadavg").read().split(maxsplit=3)
    return float(a), float(b), float(c)

def mem_used_total_mb():
    mt=ma=None
    for line in open("/proc/meminfo"):
        if line.startswith("MemTotal:"): mt=int(line.split()[1])
        elif line.startswith("MemAvailable:"): ma=int(line.split()[1])
    if mt and ma: return (mt-ma)/1024.0, mt/1024.0
    return 0.0,0.0

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="pi_proc_log.csv")
    ap.add_argument("--interval", type=float, default=60.0)
    args = ap.parse_args()

    stop=False
    def _h(*_): 
        nonlocal stop; stop=True
    signal.signal(signal.SIGINT,_h); signal.signal(signal.SIGTERM,_h)

    header = ["timestamp","uptime_s","cpu_percent","load1","load5","load15",
              "temp_c","arm_mhz","core_mhz","v3d_mhz","throttled_hex",
              "mem_used_mb","mem_total_mb"]
    new_file = not (os.path.isfile(args.out) and os.path.getsize(args.out)>0)
    prev=None
    with open(args.out,"a",newline="") as f:
        w=csv.writer(f)
        if new_file: w.writerow(header)
        while not stop:
            ts = datetime.now().isoformat(timespec="seconds")
            pct, prev = cpu_percent(prev)
            l1,l5,l15 = loadavg()
            used,total = mem_used_total_mb()
            arm, core, v3d = mhz("arm"), mhz("core"), mhz("v3d")
            thr = throttled_hex()
            up = float(open("/proc/uptime").read().split()[0])
            w.writerow([ts, f"{up:.0f}", f"{pct:.1f}", l1, l5, l15,
                        f"{temp_c():.1f}", arm, core, v3d, thr,
                        f"{used:.1f}", f"{total:.1f}"])
            f.flush()
            time.sleep(args.interval)

if __name__ == "__main__":
    main()
