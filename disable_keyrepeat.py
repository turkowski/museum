import sys, time, os

wait=0.0
if len(sys.argv)>1:
    wait = float(sys.argv[1])

time.sleep(wait)
os.system("xset -r")
    
