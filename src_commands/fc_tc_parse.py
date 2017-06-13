#!/usr/bin/env python

import sys
import csv
import json
import os

def main():
  if len(sys.argv) < 5:
    print "Usage: fc_tc_parse.py tcFile teamRoundMappingFile teamName roundNumber"
    sys.exit()

  tcFile = sys.argv[1]
  teamRoundMappingFile = sys.argv[2]
  teamName = sys.argv[3]
  roundNumber = sys.argv[4]

  teamName = teamName.lower()

  print "tcFile: %s" % tcFile
  print "teamRoundMappingFile: %s" % teamRoundMappingFile
  print "teamName: %s" % teamName
  print "roundNumber: %s" % roundNumber


  if not os.path.exists(teamRoundMappingFile) or not os.path.exists(tcFile):
    print "TC files not found"
    sys.exit()

  teamRoundMapping = None
  # get team to round mapping
  with open(teamRoundMappingFile) as teamCSVFile:
    reader = csv.DictReader(teamCSVFile)
    for row in reader:
      print(row['Team'], row['Round 1'], row['Round 2'], row['Round 3'], row['Round 4'], row['Round 5'])
      if row['Team'].lower() == teamName:
        print "Found team to round mapping"
        teamRoundMapping = row.copy()
        break
  if teamRoundMapping == None:
    print 'Not Team Round mapping found for team %s and round %s' % (teamName, roundNumber)

  targetRoundNumber = 'Round ' + roundNumber
  worldNumber = teamRoundMapping[targetRoundNumber]

  # get round to world mapping
  tcParams = None
  with open(tcFile) as tcCSVFile:
     reader = csv.DictReader(tcCSVFile)
     for row in reader:
       print(row['World'], row['T1 latency'], row['T1 uplink'], row['T1 downlink'], row['T2_3 latency'], row['T2_3 uplink'], row['T2_3 downlink'])
       if row['World'] == worldNumber:
         tcParams = row.copy()

  dataJson = json.dumps(tcParams)
  scriptDir = os.path.dirname(os.path.realpath(__file__))
  f = open(scriptDir + '/tc.cfg', 'w')
  f.write(dataJson)

if __name__ == '__main__':
  main()
