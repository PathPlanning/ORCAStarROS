from PIL import Image, ImageDraw
#from datetime import datetime
#import random
#from functools import *
#import math
#import copy
import re
from lxml import etree as ET


print("Start!")


print("Enter global path to task XML-file")
taskFileName = input()

print("Enter global path to output files (without extension)")
outputFileName = input()


mapFile = outputFileName + ".yml"
mapFilePNG = outputFileName + ".png"

print("Enter number of agents")
agNum = int(input())


print("Enable visualization? 1 -- yes, 0 -- no")
visint = int(input())
vis = False
if(visint == 1):
    vis = True

print("Enter step threshhold (-1 -- no threshhold)")
trint = int(input())

print("Enable ending? when all agent reach their goals or steps limit? 1 -- yes, 0 -- no")
endint = int(input())
endbool = False
if(endint == 1):
    endbool = True


print("Parsing XML")


tree = ET.ElementTree(file=taskFileName)
root = tree.getroot()
agents = root.find("agents")
maxAgNum = int(agents.get("number"))

if(agNum > maxAgNum):
    print("Too many agents!")
    exit(-1)


maptag = root.find("map")

cstag = maptag.find("cellsize")
resolution = float(cstag.text)

gridtag = maptag.find("grid")
griddata = []
for row in gridtag:
    rowdata = []
    for c in row.text:
        if(c == "1"):
            rowdata.append(1)
        elif(c == "0"):
            rowdata.append(0)
    griddata.append(rowdata)




h = len(griddata)
if(h < 1):
    print("Grid Error!")
    exit(-1)
    
w = len(griddata[0])
if(w < 1):
    print("Grid Error!")
    exit(-1)


print("Drawing map!")
im = Image.new('RGB', (w, h), color = 'white')
pixels = im.load()



for i in range(h):
    for j in range(w):
        if(griddata[i][j] == 1):
            pixels[j, i] = (0,0,0)
               

            
im.save(mapFilePNG, "PNG")




print("Creating map YML file!")

yml = open(mapFile, "w")
yml.write("image: " + mapFilePNG + "\n")
yml.write("resolution: " + str(resolution) + "\n")
yml.write("origin: [0.000000, 0.000000, 0.000000]\n")
yml.write("negate: 0\n")
yml.write("occupied_thresh: 0.65\n")
yml.write("free_thresh: 0.196\n")
yml.close()




print("Creating launch!")

root = ET.Element("launch")

mapserv = ET.Element("node")
mapserv.set("name", "map_server")
mapserv.set("pkg", "map_server")
mapserv.set("type", "map_server")
mapserv.set("args", mapFile)

root.append(mapserv) 

mission = ET.Element("node")
mission.set("name", "ROSMission")
mission.set("pkg", "ORCAStar")
mission.set("type", "ROSMission")
mission.set("output", "screen")

agnumparam = ET.SubElement(mission, "param")
agnumparam.set("name", "agents_number")
agnumparam.set("type", "int")
agnumparam.set("value", str(agNum))

task = ET.SubElement(mission, "param")
task.set("name", "task")
task.set("type", "string")
task.set("value", taskFileName)

thresh = ET.SubElement(mission, "param")
thresh.set("name", "threshhold")
thresh.set("type", "int")
thresh.set("value", str(trint))

end = ET.SubElement(mission, "param")
end.set("name", "end")
end.set("type", "bool")
end.set("value", str(endbool))



root.append(mission)

for i in range(agNum):
    actor = ET.Element("node")
    actor.set("name", "$(anon ROSSimActor_" + str(i) + ")")
    actor.set("pkg", "ORCAStar")
    actor.set("type", "ROSSimActor")
    root.append(actor)

    agent = ET.Element("node")
    agent.set("name", "$(anon ROSAgent_" + str(i) + ")")
    agent.set("pkg", "ORCAStar")
    agent.set("type", "ROSAgent")

    agnumparam = ET.SubElement(agent, "param")
    agnumparam.set("name", "id")
    agnumparam.set("type", "int")
    agnumparam.set("value", str(i))

    root.append(agent)


if vis:
    rviz = ET.Element("node")
    rviz.set("name", "rviz")
    rviz.set("pkg", "rviz")
    rviz.set("type", "rviz")
    root.append(rviz)


    visual = ET.Element("node")
    visual.set("name", "ROSVisualization")
    visual.set("pkg", "ORCAStar")
    visual.set("type", "ROSVisualization")
    root.append(visual)




newstr = ET.tostring(root, encoding='utf8').decode('utf8')

newstr = re.sub(">", ">\n", newstr)

f = open(outputFileName + ".launch", 'w')
f.write(newstr)
f.close()
print("Done")
