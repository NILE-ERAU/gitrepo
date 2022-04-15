import mySQL_Control as sql
import time
import datetime

Ts = 5
t = 0
last_t = 0


sql.assign_ip()


while(True):
    t = time.monotonic()
    if (t - last_t >= Ts):
        timetowait = sql.time_until()
        print(timetowait)
        if (timetowait >= 0):
            queued = sql.pull_next_command()
            command = queued[2]
            theta_q = queued[3]
            r_q = queued[4]
            z_q = queued[5]
            d0_q = queued[6]
            d1_q = queued[7]
            i0_q = queued[8]

            sql.complete_command(0,0,0,"Success")
            print("Success!")
            #
        #
        #
        last_t = t
