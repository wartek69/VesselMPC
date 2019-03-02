from collections import defaultdict
from copy import copy

LON = 0
LAT = 1
SOG = 2
HDT = 3
COG = 4
ROT = 5
RSA = 6
PilotActive = 7
ReqROT = 8
ReqRSA = 9
PilotHDT = 10
TrackROT = 11
TrackHDT = 12
XTE = 13
file = '20190225.cnr'
f = open('data/' + file)

data = defaultdict(lambda: [0] * 14)
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
    if nmeaCode == 'GGA':
      lat = nmea.split(',')[2]
      lon = nmea.split(',')[4]
      data[time][LON] = float(lon[:3]) + (float(lon[3:]) / 60)
      data[time][LAT] = float(lat[:2]) + (float(lat[2:]) / 60)
    elif nmeaCode == 'HDT':
      data[time][HDT] = nmea.split(',')[1]
    elif nmeaCode == 'VTG':
      data[time][COG] = nmea.split(',')[1]
      data[time][SOG] = nmea.split(',')[7]
    elif nmeaCode == 'ROT':
      data[time][ROT] = nmea.split(',')[1]
    elif nmeaCode == 'RSA':
      data[time][RSA] = nmea.split(',')[1]
    elif nmeaCode == 'HTD':
      data[time][PilotActive] = int(nmea.split(',')[5] == 'T')
      data[time][PilotHDT] = nmea.split(',')[17][:-3]
      if len(nmea.split(',')[9]) > 0:
        data[time][ReqROT] = float(nmea.split(',')[9])
      else:
        data[time][ReqROT] = 0
      if nmea.split(',')[3] == 'L':
        data[time][ReqRSA] = -float(nmea.split(',')[2])
      else:
        data[time][ReqRSA] = float(nmea.split(',')[2])
    elif nmeaCode == 'XTE':
      data[time][XTE] = float(nmea.split(',')[3]) * 1852
    elif nmeaCode == 'HTC':
      data[time][TrackROT] = nmea.split(',')[9]
      data[time][TrackHDT] = nmea.split(',')[10]

  lastTime = time
f.close();
f = open('output/'+file, 'w+')
f.write('Time,Lon,Lat,Speed,Heading,Course,ROT,Rudder,PilotActive,ReqROT,ReqRudder,HeadingInPilot,TrackROT,TrackCourse,XTE\n')
print('Time,Lon,Lat,Speed,Heading,Course,ROT,Rudder,PilotActive,ReqROT,ReqRudder,HeadingInPilot,TrackROT,TrackCourse,XTE')
for time in data:
  print('{},{:.7f},{:.7f},{},{},{},{},{},{},{},{},{},{},{},{:.1f}'.format(
    time,
    data[time][LON],
    data[time][LAT],
    data[time][SOG],
    data[time][HDT],
    data[time][COG],
    data[time][ROT],
    data[time][RSA],
    data[time][PilotActive],
    data[time][ReqROT],
    data[time][ReqRSA],
    data[time][PilotHDT],
    data[time][TrackROT],
    data[time][TrackHDT],
    data[time][XTE]
  ))
  f.write('{},{:.7f},{:.7f},{},{},{},{},{},{},{},{},{},{},{},{:.1f}\n'.format(
      time,
      data[time][LON],
      data[time][LAT],
      data[time][SOG],
      data[time][HDT],
      data[time][COG],
      data[time][ROT],
      data[time][RSA],
      data[time][PilotActive],
      data[time][ReqROT],
      data[time][ReqRSA],
      data[time][PilotHDT],
      data[time][TrackROT],
      data[time][TrackHDT],
      data[time][XTE]
  ))
