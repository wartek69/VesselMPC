from collections import defaultdict
from copy import copy

LON = 0
LAT = 1
SOG = 2
HDT = 3
COG = 4
EIROT = 5
ReqROT = 6
GPROT = 7
file = 'Vlieland.nmea'
f = open(file)

data = defaultdict(lambda: [0] * 8)
lastTime = 0
for line in f.readlines():
    if len(line) > 0 and line[0] == '#':
        continue

    line = line.strip().split()
    time = line[2]
    if time != lastTime and lastTime != 0:
        data[time] = copy(data[lastTime])

    nmea = line[4]
    if len(nmea) > 3 and nmea[-3] == '*':
        nmeaCode = nmea[3:6]
        if nmeaCode == 'HDT':
            data[time][HDT] = nmea.split(',')[1]
        elif nmeaCode == 'ROT':
            if nmea[1:3] == 'EI':
                data[time][EIROT] = nmea.split(',')[1]
            elif nmea[1:3] == 'GP':
                data[time][GPROT] = nmea.split(',')[1]
        elif nmeaCode == 'RMC':
            sog = nmea.split(',')[7]
            status = nmea.split(',')[2]
            if sog == '000.0' or sog == '0.0' or status == 'V':
                pass
            else:
                cog = nmea.split(',')[8]
                lat = nmea.split(',')[3]
                lon = nmea.split(',')[5]

                data[time][SOG] = sog
                data[time][COG] = cog
                data[time][LON] = float(lon[:3]) + (float(lon[3:]) / 60)
                data[time][LAT] = float(lat[:2]) + (float(lat[2:]) / 60)
        elif nmeaCode == 'TPS':
            rrot = nmea.split(',')[4]
            data[time][ReqROT] = rrot

    lastTime = time
f.close()

f = open('output/' + file + 'outall', 'w+')
f.write('Time,Lon,Lat,Speed,Heading,Course,EIROT,GPROT,ReqROT\n')
print('Time,Lon,Lat,Speed,Heading,Course,EIROT,GPROT,ReqROT\n')
for time in data:
    print('{},{:.7f},{:.7f},{},{},{},{},{},{}'.format(
        time,
        data[time][LON],
        data[time][LAT],
        data[time][SOG],
        data[time][HDT],
        data[time][COG],
        data[time][EIROT],
        data[time][GPROT],
        data[time][ReqROT]
    ))
    f.write('{},{:.7f},{:.7f},{},{},{},{},{},{}\n'.format(
        time,
        data[time][LON],
        data[time][LAT],
        data[time][SOG],
        data[time][HDT],
        data[time][COG],
        data[time][EIROT],
        data[time][GPROT],
        data[time][ReqROT]
    ))
f.close()
