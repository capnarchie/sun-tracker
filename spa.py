from cmath import acos, sin, tan
import math

from numpy import rad2deg
#TODO: translate to C

current_latitude = 58.378025
current_longtitude = 26.728493

#get local time from rtcc as variable
local_time = 0
#get days since the start of year from rtcc as variable
days_since_start_of_year = 0
def LSTM(): # 15 * local time - UTC in hours aka 15*current timezone
    return 15*3

def eot(days_since_start_of_year):
    B = (360/365)*(days_since_start_of_year - 81)
    return 9.87 * math.sin(2*B) - 7.53*math.cos(B) - 1.5 * math.sin(B)

def TC(local_standard_time_meridian, longtitude, eq_of_time):
    return 4*(longtitude - local_standard_time_meridian) + eq_of_time
    


def LST(local_time, time_correction):
    return local_time+time_correction/60
    

def HRA(LST):
    return 15*(LST-12) #deg
    


# placeholder currently figure out formula for this
declination_angle = rad2deg(-23.45*math.cos((360/365)*(days_since_start_of_year+10)))

#sunrise = 12 - (1/15) * rad2deg(acos(-tan(current_latitude)*tan(declination_angle))) - TC(LSTM(),current_longtitude, eot(days_since_start_of_year))
#sunset = 12 + (1/15) * rad2deg(acos(-tan(current_latitude)*tan(declination_angle))) - TC(LSTM(),current_longtitude, eot(days_since_start_of_year))

#angle = rad2deg(math.asin(sin(declination_angle)*sin(current_latitude) + math.cos(declination_angle)*math.cos(current_latitude)*math.cos(HRA(LST(local_time, TC(LSTM(),current_longtitude, eot(days_since_start_of_year)))))))


while True:
    
    angle = rad2deg(math.asin(sin(declination_angle)*sin(current_latitude) + math.cos(declination_angle)*math.cos(current_latitude)*math.cos(HRA(LST(local_time, TC(LSTM(),current_longtitude, eot(days_since_start_of_year)))))))
    if angle > 0: # add comparison to our positional reference
        # move motors closer to the sun
        # put cpu to sleep for 15 min
        pass
    
    if angle <= 0:
        #calculate new sunrise and put CPU to deep sleep
        sunrise = 12 - (1/15) * rad2deg(acos(-tan(current_latitude)*tan(declination_angle))) - TC(LSTM(),current_longtitude, eot(days_since_start_of_year))